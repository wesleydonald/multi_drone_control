"""Shared two/four-drone IRL launch; the default remains the two-drone baseline."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tejen_mission.m2_irl_launch_config import (
    gate_profile_ros_parameters,
    identity_ros_parameters,
    load_hardware_config,
    parse_gate_profiles,
    progressive_ros_parameters,
    parse_fleet_identity,
    parse_flight_order,
    validate_mission_hardware_config,
    parse_irl_ring_geometry,
)


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
    # Mission validation checks all props_off_verified flags before nodes exist.
    payload = load_hardware_config(config_path)
    identity = parse_fleet_identity(payload)
    vehicle_ids = tuple(identity.vehicles)
    flight_order = parse_flight_order(payload)
    expected_count = int(LaunchConfiguration("expected_vehicle_count", default="2").perform(context))
    if len(vehicle_ids) != expected_count:
        raise ValueError(f"M2 IRL vehicle count {len(vehicle_ids)} does not match requested {expected_count}")
    profiles = parse_gate_profiles(payload)
    progressive = progressive_ros_parameters(payload)
    identity_params = identity_ros_parameters(identity)
    profile_params = gate_profile_ros_parameters(profiles)
    root = payload["m2_irl"]
    ring_geometry = parse_irl_ring_geometry(payload)
    vehicles = identity.vehicles
    geometry = root["geometry"]
    if mode == "mission":
        validate_mission_hardware_config(payload)
    log_dir = Path(LaunchConfiguration("log_dir").perform(context)).expanduser()
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "resolved_config.yaml").write_text(yaml.safe_dump(payload, sort_keys=False))

    mocap = root["mocap"]
    actions = [
        LogInfo(msg=f"M2 IRL {mode}: manual ARM/TAKEOFF/LAND authority only. "
                    + ", ".join(f"{v}=QUAD{vehicles[v].physical_quad}" for v in vehicle_ids)
                    + f"; flight order: {' -> '.join(flight_order)}"),
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
    for vehicle_id in vehicle_ids:
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
                "readiness_mocap_timeout_s": profiles.mocap_timeout_s,
                "readiness_telemetry_timeout_s": profiles.telemetry_timeout_s,
                "readiness_ring_timeout_s": profiles.ring_timeout_s,
                "readiness_link_statistics_timeout_s": profiles.link_statistics_timeout_s,
                "status_timeout_s": profiles.status_timeout_s,
                "stationary_dwell_s": profiles.ground_dwell_s,
                "max_stationary_speed_mps": profiles.ground_speed_mps,
                **identity_params,
                **profile_params,
                "tether_length_m": float(geometry["tether_length_m"]),
                "ring_plate_pitch_diameter_m": ring_geometry.plate_pitch_diameter_m,
                "ring_plate_diameter_m": ring_geometry.plate_diameter_m,
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
                "vehicle_ids": list(flight_order),
                "target_attachment_count": len(vehicle_ids),
                "operator_hold_after_goal": True,
                **progressive["supervisor"],
                "attachment_evidence_timeout_s": 0.75,
                "operator_land_request_topic": "/m2d/operator/land_request",
            }],
        ),
    ])

    dynamic_share = Path(get_package_share_directory("tejen_dynamic_planner"))
    commissioned_backend = _backend_parameters(dynamic_share)
    contact = root["attachment_detector"]
    search = profiles.attachment_search
    # The YAML tether is anchor-to-face; this legacy planner fallback is
    # body-to-marker centre. Keep both derived from the same physical geometry.
    body_to_magnet_center_m = (
        float(geometry["tether_length_m"])
        - float(geometry["tether_anchor_body"][2])
        + float(contact["magnet_contact_translation_m"][2])
    )
    if not math.isfinite(body_to_magnet_center_m) or body_to_magnet_center_m <= 0.0:
        raise ValueError("M2 IRL body-to-magnet-centre distance must be finite and positive")
    observer_nodes = []
    planner_nodes = []
    backend_nodes = []
    controller_nodes = []
    for index, vehicle_id in enumerate(vehicle_ids):
        peers = [v for v in vehicle_ids if v != vehicle_id]
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
                "carrier_frame_leaf": "base_link",
                "magnet_frame_leaf": "magnet_tip_link",
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
                "candidate_normal_m": search.candidate_normal_m,
                "candidate_xy_m": search.candidate_xy_m,
                "proof_normal_m": search.proof_normal_m,
                "proof_speed_mps": search.proof_relative_speed_mps,
                "ring_plate_pitch_diameter_m": ring_geometry.plate_pitch_diameter_m,
                "ring_plate_diameter_m": ring_geometry.plate_diameter_m,
                "proof_max_contact_drift_m": search.proof_max_contact_drift_m,
                "contact_offset_x": float(contact["magnet_contact_translation_m"][0]),
                "contact_offset_y": float(contact["magnet_contact_translation_m"][1]),
                "contact_offset_z": float(contact["magnet_contact_translation_m"][2]),
                "contact_rotation_qx": float(contact["magnet_contact_quaternion_xyzw"][0]),
                "contact_rotation_qy": float(contact["magnet_contact_quaternion_xyzw"][1]),
                "contact_rotation_qz": float(contact["magnet_contact_quaternion_xyzw"][2]),
                "contact_rotation_qw": float(contact["magnet_contact_quaternion_xyzw"][3]),
                "log_dir": str(log_dir / vehicle_id / "attachment"),
                "tether_anchor_body": list(geometry["tether_anchor_body"]),
                **progressive["observer"],
                "initial_contact_max_normal_m": 0.008,
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
                "m2b_settle_position_tolerance_m": profiles.airborne_position_tolerance_m,
                "m2b_settle_speed_mps": profiles.airborne_speed_mps,
                "m2b_settle_swing_deg": profiles.airborne_swing_deg,
                "m2b_takeoff_settle_swing_deg": profiles.takeoff_swing_deg,
                "m2b_settle_dwell_s": profiles.airborne_dwell_s,
                "m2b_irl_tip_alignment_enabled": True,
                "m2b_irl_capture_search_enabled": True,
                "m2b_contact_overtravel_m": search.contact_overtravel_m,
                "m2b_irl_max_contact_correction_m": search.max_contact_correction_m,
                "m2b_irl_contact_correction_rate_mps": search.contact_correction_rate_mps,
                "m2b_capture_hold_normal_m": search.capture_normal_m,
                **{f"m2b_irl_tip_{key}": value for key, value in asdict(profiles.tip_alignment).items()},
                "m2b_pendulum_timeout_s": profiles.mocap_timeout_s,
                "m2b_ring_pose_timeout_s": profiles.ring_timeout_s,
                "m2b_link_statistics_timeout_s": profiles.link_statistics_timeout_s,
                "c1f2_prepare_max_lift_drift_m": profiles.c1f2_max_lift_drift_m,
                "c1f2_prepare_settle_speed_mps": profiles.c1f2_speed_mps,
                "c1f2_prepare_settle_acceleration_mps2": profiles.c1f2_acceleration_mps2,
                "c1f2_prepare_settle_dwell_s": profiles.c1f2_dwell_s,
                **profile_params,
                "vehicle_id": vehicle_id,
                "mission_mode": "m2b",
                "m2b_commissioning_stage": "b2",
                "c1f1_shadow_transfer_enabled": True,
                "c1f2_cpp_authority_enabled": True,
                "c1f1_shadow_target_topic": f"{ns}/dynamic_planner/transfer_target",
                "c1f1_shadow_enable_topic": f"{ns}/dynamic_planner/transfer_enable",
                "c1f1_shadow_reference_topic": f"{ns}/dynamic_planner/transfer_reference",
                "c1f1_shadow_status_topic": f"{ns}/dynamic_planner/transfer_status",
                "c1f2_target_velocity_topic": f"{ns}/dynamic_planner/transfer_target_velocity",
                "c1f2_authority_topic": f"{ns}/dynamic_planner/transfer_authority",
                "c1f2_authority_ack_topic": f"{ns}/dynamic_planner/transfer_authority_ack",
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
                "ring_plate_pitch_diameter_m": ring_geometry.plate_pitch_diameter_m,
                "ring_plate_diameter_m": ring_geometry.plate_diameter_m,
                "m2b_pendulum_topic": f"{ns}/pendulum_swing_state",
                "m2b_attached_hold_angle_deg": 15.0,
                "m2b_anchor_to_contact_length_m": float(geometry["tether_length_m"]),
                "m2b_tether_anchor_body_x": float(geometry["tether_anchor_body"][0]),
                "m2b_tether_anchor_body_y": float(geometry["tether_anchor_body"][1]),
                "m2b_tether_anchor_body_z": float(geometry["tether_anchor_body"][2]),
                "magnet_drop_below_quad": body_to_magnet_center_m,
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
                **progressive["planner"],
                "m2b_irl_quiet_miss_enabled": True,
                "m2b_irl_quiet_miss_wait_s": profiles.quiet_miss.wait_s,
                "m2b_irl_quiet_miss_improvement_m": profiles.quiet_miss.improvement_m,
                "m2b_irl_quiet_miss_relative_speed_mps": profiles.quiet_miss.relative_speed_mps,
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
            # The first passive solve needs the commissioned 100 ms future splice
            # while peer/ring commitments roll forward during the safety recheck.
            # This seed never grants execution authority by itself.
            "seed_stationary_on_activation": True,
            "cooperative_scene_enabled": True,
            "cooperative_prediction_mode": "shared_trajectory",
            "cooperative_shared_start_skew_s": profiles.cooperative_shared_start_skew_s,
            "cooperative_shared_start_skew_max_speed_mps": profiles.ground_speed_mps,
            "cooperative_drone_state_topics": [f"/{peer}/motion_capture_state" for peer in peers],
            "cooperative_committed_trajectory_topics": [f"/{peer}/committed_trajectory" for peer in peers],
            "cooperative_vehicle_ids": peers,
            "cooperative_attached_plate_topics": [f"/{peer}/m2d/attached_plate_id" for peer in peers],
            "cooperative_ring_pose_topic": "/m2/ring/pose",
            "cooperative_ring_pose_index": 0,
            "moving_basket_scene_enabled": True,
            "moving_basket_committed_trajectory_topic": "/m2d/ring/committed_trajectory",
            "moving_basket_collision_mode": "segmented_ring",
            "cooperative_ring_plate_pitch_diameter_m": ring_geometry.plate_pitch_diameter_m,
            "moving_ring_segment_center_radius_m": ring_geometry.plate_pitch_radius_m,
            "moving_ring_segment_radial_width_m": ring_geometry.radial_width_m,
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
    ])
    actions.extend(TimerAction(period=2.0 + 2.0 * i, actions=[node])
                   for i, node in enumerate(controller_nodes))
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
        DeclareLaunchArgument("expected_vehicle_count", default_value="2"),
        DeclareLaunchArgument(
            "config_file", default_value=str(share / "config" / "m2_irl_two_drone.yaml")
        ),
        DeclareLaunchArgument("log_dir", default_value="logs/m2_irl/manual"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
