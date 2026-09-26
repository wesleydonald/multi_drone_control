"""Minimum two-drone M2 IRL launch. It never publishes ARM or TAKEOFF commands."""

from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tejen_mission.m2_irl_launch_config import (
    identity_ros_parameters,
    load_hardware_config,
    parse_two_drone_identity,
    validate_mission_hardware_config,
)


VEHICLE_IDS = ("drone_0", "drone_1")
VALID_MODES = ("io", "mission")


def _backend_parameters(dynamic_share: Path) -> dict:
    path = dynamic_share / "config" / "c1f6_moving_rendezvous.yaml"
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    return deepcopy(payload["dynamic_planner_transfer_backend"]["ros__parameters"])


def _launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in {"io", "mission"}:  # valid values: "io", "mission"
        raise RuntimeError(f"mode must be one of {VALID_MODES}, got {mode!r}")
    config_path = Path(LaunchConfiguration("config_file").perform(context)).expanduser()
    # Mission validation checks both props_off_verified flags before nodes exist.
    payload = load_hardware_config(config_path)
    identity = parse_two_drone_identity(payload)
    identity_params = identity_ros_parameters(identity)
    root = payload["m2_irl"]
    vehicles = identity.vehicles
    geometry = root["geometry"]
    if mode == "mission":
        validate_mission_hardware_config(payload)
    log_dir = Path(LaunchConfiguration("log_dir").perform(context)).expanduser()
    log_dir.mkdir(parents=True, exist_ok=True)

    mocap = root["mocap"]
    actions = [
        LogInfo(msg=f"M2 IRL {mode}: manual ARM/TAKEOFF/LAND authority only."),
        Node(
            package="tejen_mission", executable="m2_irl_mocap_router",
            name="m2_irl_mocap_router", output="screen",
            parameters=[{
                "use_sim_time": False,
                "bind_address": str(mocap.get("bind_address", "0.0.0.0")),
                "bind_port": int(mocap.get("bind_port", 1511)),
                "max_rate_hz": float(mocap.get("max_rate_hz", 120.0)),
                "tether_anchor_x": float(geometry["tether_anchor_body"][0]),
                "tether_anchor_y": float(geometry["tether_anchor_body"][1]),
                "tether_anchor_z": float(geometry["tether_anchor_body"][2]),
                **identity_params,
                "ring_marker_translation_x": float(geometry["ring_marker_translation_m"][0]),
                "ring_marker_translation_y": float(geometry["ring_marker_translation_m"][1]),
                "ring_marker_translation_z": float(geometry["ring_marker_translation_m"][2]),
                "ring_marker_rotation_qx": float(geometry["ring_marker_quaternion_xyzw"][0]),
                "ring_marker_rotation_qy": float(geometry["ring_marker_quaternion_xyzw"][1]),
                "ring_marker_rotation_qz": float(geometry["ring_marker_quaternion_xyzw"][2]),
                "ring_marker_rotation_qw": float(geometry["ring_marker_quaternion_xyzw"][3]),
            }],
        ),
    ]
    for vehicle_id in VEHICLE_IDS:
        vehicle = vehicles[vehicle_id]
        actions.append(Node(
            package="tejen_mission", executable="elrs_interface_irl",
            namespace=vehicle_id, name="elrs_interface_irl", output="screen",
            parameters=[{
                "use_sim_time": False,
                "serial_port": vehicle.elrs_device,
                "magnet_channels": [vehicle.magnet_channel],
                "magnet_command_topic": f"/{vehicle_id}/magnet/ELRSCommand",
                "magnet_string_command_topic": "magnet/command",
            }],
        ))
    if mode == "io":
        return actions

    actions.extend([
        Node(
            package="tejen_mission", executable="m2_irl_fleet_manager",
            name="m2_irl_fleet_manager", output="screen",
            parameters=[{
                "use_sim_time": False,
                "readiness_timeout_s": 0.50,
                "stationary_dwell_s": 1.0,
                "status_timeout_s": 1.25,
                "max_stationary_speed_mps": 0.05,
                **identity_params,
                "tether_length_m": float(geometry["tether_length_m"]),
                "tether_anchor_body": list(geometry["tether_anchor_body"]),
            }],
        ),
        Node(
            package="tejen_mission", executable="m2d_ring_commitment",
            name="m2d_ring_commitment", output="screen",
            parameters=[{
                "use_sim_time": False,
                "ring_pose_topic": "/m2/ring/pose",
                "ring_pose_index": 0,
                "committed_trajectory_topic": "/m2d/ring/committed_trajectory",
                "vehicle_id": "m2d_ring",
                "ring_pose_timeout_s": 0.50,
                "commitment_horizon_s": 10.0,
            }],
        ),
        Node(
            package="tejen_mission", executable="m2d_fleet_supervisor",
            name="m2d_fleet_supervisor", output="screen",
            parameters=[{
                "use_sim_time": False,
                "vehicle_ids": list(VEHICLE_IDS),
                "target_attachment_count": 2,
                "operator_hold_after_goal": True,
                "attachment_evidence_timeout_s": 0.75,
                "operator_land_request_topic": "/m2d/operator/land_request",
            }],
        ),
    ])

    dynamic_share = Path(get_package_share_directory("tejen_dynamic_planner"))
    commissioned_backend = _backend_parameters(dynamic_share)
    proof_angle = math.radians(15.0)
    contact = root["attachment_detector"]
    observer_nodes = []
    planner_nodes = []
    backend_nodes = []
    controller_nodes = []
    for index, vehicle_id in enumerate(VEHICLE_IDS):
        peer = VEHICLE_IDS[1 - index]
        ns = f"/{vehicle_id}"
        vehicle_log_dir = log_dir / vehicle_id
        vehicle_log_dir.mkdir(parents=True, exist_ok=True)
        observer_nodes.append(Node(
            package="tejen_mission", executable="m2_attachment_observer",
            namespace=vehicle_id, name="m2_attachment_observer", output="screen",
            parameters=[{
                "use_sim_time": False,
                "pose_source": "tf_named",
                "carrier_tf_topic": f"{ns}/mocap/transforms",
                "carrier_frame_leaf": f"{vehicle_id}/base_link",
                "magnet_frame_leaf": f"{vehicle_id}/magnet_tip_link",
                "ring_pose_source": "pose_array",
                "ring_pose_topic": "/m2/ring/pose",
                "ring_pose_index": 0,
                "assigned_plate_topic": f"{ns}/m2d/assigned_plate_id",
                "magnet_command_topic": f"{ns}/magnet/command",
                "proof_requested_topic": f"{ns}/attachment/proof_requested",
                "reset_topic": f"{ns}/attachment/reset",
                "diagnostics_topic": f"{ns}/attachment/diagnostics",
                "state_topic": f"{ns}/attachment/state",
                "confirmed_topic": f"{ns}/attachment/confirmed",
                "probe_phase_topic": f"{ns}/attachment/probe_phase",
                "magnet_off_counts_as_loss": bool(contact["magnet_off_counts_as_loss"]),
                "contact_offset_x": float(contact["magnet_contact_translation_m"][0]),
                "contact_offset_y": float(contact["magnet_contact_translation_m"][1]),
                "contact_offset_z": float(contact["magnet_contact_translation_m"][2]),
                "contact_rotation_qx": float(contact["magnet_contact_quaternion_xyzw"][0]),
                "contact_rotation_qy": float(contact["magnet_contact_quaternion_xyzw"][1]),
                "contact_rotation_qz": float(contact["magnet_contact_quaternion_xyzw"][2]),
                "contact_rotation_qw": float(contact["magnet_contact_quaternion_xyzw"][3]),
                "proof_direction_radial": math.sin(proof_angle),
                "proof_direction_tangential": 0.0,
                "proof_direction_normal": math.cos(proof_angle),
                "log_dir": str(log_dir / vehicle_id / "attachment"),
            }],
        ))
        planner_nodes.append(Node(
            package="tejen_mission", executable="online_join_planner",
            namespace=vehicle_id, name="online_join_planner", output="screen",
            parameters=[{
                "use_sim_time": False,
                "min_reference_z": 0.05,
                "takeoff_height": 0.65,
                "m2b_takeoff_require_magnet_clearance": True,
                "m2b_takeoff_magnet_clearance_m": 0.03,
                "m2b_takeoff_hover_timeout_s": 15.0,
                "vehicle_id": vehicle_id,
                "mission_mode": "m2b",
                "m2b_commissioning_stage": "b2",
                "m2d_enabled": True,
                "m2d_mission_permission_topic": f"{ns}/m2d/mission_permission",
                "m2d_assigned_plate_topic": f"{ns}/m2d/assigned_plate_id",
                "m2b_attachment_truth_source": "measured_detector",
                "m2b_require_recent_link_statistics": True,
                "m2b_bootstrap_release_required": False,
                "m2b_ring_pose_source": "pose_array",
                "m2b_ring_pose_topic": "/m2/ring/pose",
                "m2b_ring_pose_index": 0,
                "m2b_b1_attachment_enabled": True,
                "m2b_b1_proof_enabled": True,
                "m2b_arm_permission_topic": f"{ns}/join_planner/arm_permission",
                "m2b_mpc_mode_topic": f"{ns}/join_planner/mpc_mode",
                "m2b_arming_state_topic": f"{ns}/arming_state_feedback",
                "m2b_attachment_diagnostics_topic": f"{ns}/attachment/diagnostics",
                "m2b_proof_requested_topic": f"{ns}/attachment/proof_requested",
                "m2b_detector_reset_topic": f"{ns}/attachment/reset",
                "m2b_pendulum_topic": f"{ns}/pendulum_swing_state",
                "m2b_proof_direction_radial": math.sin(proof_angle),
                "m2b_proof_direction_tangential": 0.0,
                "m2b_proof_direction_normal": math.cos(proof_angle),
                "m2b_attached_hold_angle_deg": 15.0,
                "m2b_anchor_to_contact_length_m": float(geometry["tether_length_m"]),
                "m2b_tether_anchor_body_x": float(geometry["tether_anchor_body"][0]),
                "m2b_tether_anchor_body_y": float(geometry["tether_anchor_body"][1]),
                "m2b_tether_anchor_body_z": float(geometry["tether_anchor_body"][2]),
                "use_measured_magnet_tip": True,
                "drone_state_topic": f"{ns}/motion_capture_state",
                "magnet_tip_pose_topic": f"{ns}/magnet_tip_pose",
                "reference_topic": f"{ns}/join_planner/reference",
                "phase_topic": f"{ns}/join_planner/phase",
                "marker_topic": f"{ns}/join_planner/markers",
                "drone_command_topic": f"{ns}/command",
                "magnet_command_topic": f"{ns}/magnet/command",
                "external_landing_topic": f"{ns}/join_planner/land_now",
                "publish_committed_trajectory": True,
                "committed_trajectory_topic": f"{ns}/committed_trajectory",
                "enable_csv_logging": True,
                "log_directory": str(log_dir / vehicle_id / "join_planner"),
            }],
        ))
        backend = deepcopy(commissioned_backend)
        backend.update({
            "use_sim_time": False,
            "vehicle_id": vehicle_id,
            "state_topic": f"{ns}/motion_capture_state",
            "target_topic": f"{ns}/dynamic_planner/transfer_target",
            "target_velocity_topic": f"{ns}/dynamic_planner/transfer_target_velocity",
            "enable_topic": f"{ns}/dynamic_planner/transfer_enable",
            "authority_topic": f"{ns}/dynamic_planner/transfer_authority",
            "authority_ack_topic": f"{ns}/dynamic_planner/transfer_authority_ack",
            "shadow_reference_topic": f"{ns}/dynamic_planner/transfer_reference",
            "diagnostics_topic": f"{ns}/dynamic_planner/transfer_status",
            "marker_topic": f"{ns}/dynamic_planner/markers",
            "cooperative_scene_enabled": True,
            "cooperative_prediction_mode": "shared_trajectory",
            "cooperative_drone_state_topics": [f"/{peer}/motion_capture_state"],
            "cooperative_committed_trajectory_topics": [f"/{peer}/committed_trajectory"],
            "cooperative_vehicle_ids": [peer],
            "cooperative_attached_plate_topics": [f"/{peer}/m2d/attached_plate_id"],
            "cooperative_ring_pose_topic": "/m2/ring/pose",
            "cooperative_ring_pose_index": 0,
            "moving_basket_scene_enabled": True,
            "moving_basket_committed_trajectory_topic": "/m2d/ring/committed_trajectory",
            "moving_basket_collision_mode": "segmented_ring",
            "static_scene_enabled": False,
            "payload_collision_enabled": False,
            "csv_path": str(log_dir / vehicle_id / "backend.csv"),
            "replan_csv_path": str(log_dir / vehicle_id / "replans.csv"),
        })
        backend_nodes.append(Node(
            package="tejen_dynamic_planner", executable="dynamic_planner_transfer_backend",
            namespace=vehicle_id, name="dynamic_planner_transfer_backend", output="screen",
            parameters=[backend],
        ))
        controller_nodes.append(Node(
            package="tejen_mpc", executable="main",
            namespace=vehicle_id, name="tejen_mpc", output="screen",
            cwd=str(log_dir / vehicle_id),
            remappings=[
                ("drone_arming_service", "arming_service"),
                ("drone_command", "command"),
                ("drone_arming_state_feedback", "arming_state_feedback"),
            ],
            parameters=[{
                "use_sim_time": False,
                "use_external_reference": True,
                "disarmed_external_reference_fast_path": True,
                "external_reference_topic": f"{ns}/join_planner/reference",
                "require_external_arm_permission": True,
                "external_arm_permission_topic": f"{ns}/join_planner/arm_permission",
                "arming_state_feedback_period_s": 0.1,
                "enable_thrust_ratio_ukf": True,
                "thrust_ratio_estimator_backend": "full_model_kt_ukf",
                "thrust_ratio_estimator_rate_hz": 10.0,
                "enable_thrust_ratio_feedback": True,
                "thrust_ratio_feedback_min_updates": 15,
                "thrust_ratio_feedback_max_std": 1.5,
                "thrust_ratio_feedback_rate_per_s": 0.5,
                "thrust_ratio_feedback_max_fractional_change": 0.25,
                "thrust_ratio_feedback_deadband": 0.20,
                "payload_mpc_mode_topic": f"{ns}/join_planner/mpc_mode",
                "payload_mpc_mode_status_topic": "tejen_mpc/mpc_mode_status",
                "c1d_status_topic": "tejen_mpc/c1d_status",
                "acados_cache_dir": str(log_dir / vehicle_id / "acados_cache"),
                "logging_instance": vehicle_id,
            }],
        ))

    actions.extend([
        TimerAction(period=0.5, actions=observer_nodes),
        TimerAction(period=1.0, actions=backend_nodes),
        TimerAction(period=1.5, actions=planner_nodes),
        TimerAction(period=2.0, actions=[controller_nodes[0]]),
        TimerAction(period=4.0, actions=[controller_nodes[1]]),
    ])
    if LaunchConfiguration("rviz").perform(context).strip().lower() == "true":
        visualisation_share = get_package_share_directory("drone_visualisation")
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="m2d_rviz",
            output="screen",
            arguments=[
                "-d", str(Path(visualisation_share) / "rviz" / "m2d_four_drone.rviz")
            ],
            parameters=[{"use_sim_time": False}],
        ))
    return actions


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("tejen_mission"))
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="io"),
        DeclareLaunchArgument(
            "config_file", default_value=str(share / "config" / "m2_irl_two_drone.yaml")
        ),
        DeclareLaunchArgument("log_dir", default_value="logs/m2_irl/manual"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
