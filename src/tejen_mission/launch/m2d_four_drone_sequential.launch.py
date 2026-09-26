"""M2D sequential four-drone attachment topology.

This launch composes the proven M2C four-X3 plumbing with four namespaced M2B B2
attachment executives.  Controllers remain separately launched by the supervised
runner so ACADOS startup retains the known health gates.  Fleet progression is
owned only by m2d_fleet_supervisor; operator ARM/TAKEOFF remains manual.
"""

from copy import deepcopy
import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DRONE_IDS = (0, 1, 2, 3)

# Authoritative M2D planner copy of the physical ring-bar geometry in
# simulation_assets/m2a_ring_fixture.sdf. A source-contract test parses that SDF
# so planner and Gazebo dimensions cannot silently drift apart.
M2D_RING_SEGMENT_COUNT = 24
M2D_RING_SEGMENT_CENTER_RADIUS_M = 0.25
M2D_RING_SEGMENT_TANGENTIAL_LENGTH_M = 0.068067840828
M2D_RING_SEGMENT_RADIAL_WIDTH_M = 0.060
M2D_RING_SEGMENT_HEIGHT_M = 0.030
M2D_RING_SEGMENT_CENTER_Z_OFFSET_M = -0.020
M2D_RING_SEGMENT_PADDING_M = 0.005
VEHICLE_IDS = tuple(f"drone_{i}" for i in DRONE_IDS)


def _load_commissioned_backend_params(dynamic_share: Path) -> dict:
    """Load the commissioned C1F.6 tuning independent of ROS node namespace.

    Passing the YAML filename directly to a namespaced backend does not apply a
    top-level ``dynamic_planner_transfer_backend`` block to
    ``/drone_i/dynamic_planner_transfer_backend`` on ROS 2 Humble.  Extract the
    ros__parameters mapping explicitly so every M2D backend inherits the same
    commissioned tuning before its per-drone topic/tether overrides are layered
    on top.
    """
    config = dynamic_share / "config" / "c1f6_moving_rendezvous.yaml"
    data = yaml.safe_load(config.read_text())
    try:
        params = data["dynamic_planner_transfer_backend"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError(
            f"Invalid commissioned C1F.6 backend parameter file: {config}"
        ) from exc
    if not isinstance(params, dict):
        raise RuntimeError(
            f"Commissioned C1F.6 ros__parameters is not a mapping: {config}"
        )
    return params


def _m2b_params(drone_id: int, log_root):
    ns = f"drone_{drone_id}"
    return {
        "use_sim_time": True,
        "vehicle_id": ns,
        "assigned_attachment_id": f"attachment_{drone_id}",
        "mission_mode": "m2b",
        "m2b_commissioning_stage": "b2",
        "c1f1_shadow_transfer_enabled": True,
        "c1f2_cpp_authority_enabled": True,
        "c1f1_shadow_target_topic": f"/{ns}/dynamic_planner/transfer_target",
        "c1f1_shadow_enable_topic": f"/{ns}/dynamic_planner/transfer_enable",
        "c1f1_shadow_reference_topic": f"/{ns}/dynamic_planner/transfer_reference",
        "c1f1_shadow_status_topic": f"/{ns}/dynamic_planner/transfer_status",
        "c1f2_target_velocity_topic": f"/{ns}/dynamic_planner/transfer_target_velocity",
        "c1f2_authority_topic": f"/{ns}/dynamic_planner/transfer_authority",
        "c1f2_authority_ack_topic": f"/{ns}/dynamic_planner/transfer_authority_ack",
        "m2d_enabled": True,
        "m2d_mission_permission_topic": f"/{ns}/m2d/mission_permission",
        "m2d_transit_owner_topic": "/m2d/transit_owner",
        "m2d_assigned_plate_topic": f"/{ns}/m2d/assigned_plate_id",
        "m2b_ring_pose_source": "pose_array",
        "m2b_ring_pose_topic": "/model/payload_model/pose",
        "m2b_ring_pose_index": 1,
        "m2b_ring_pose_timeout_s": 0.50,
        "m2b_b1_attachment_enabled": True,
        "m2b_b1_proof_enabled": True,
        "m2b_assigned_plate_id": 0,
        "m2b_joint_truth_topic": f"/{ns}/magnet/joint_detached_truth",
        "m2b_bootstrap_release_required": True,
        "m2b_bootstrap_release_topic": f"/{ns}/m2b/bootstrap/release",
        "m2b_raw_detach_request_topic": f"/{ns}/m2b/sim/raw_detach_request",
        "m2b_arm_permission_topic": f"/{ns}/join_planner/arm_permission",
        "m2b_mpc_mode_topic": f"/{ns}/join_planner/mpc_mode",
        "m2b_arming_state_topic": f"/{ns}/arming_state_feedback",
        "m2b_attachment_diagnostics_topic": f"/{ns}/attachment/diagnostics",
        "m2b_proof_requested_topic": f"/{ns}/attachment/proof_requested",
        "m2b_detector_reset_topic": f"/{ns}/attachment/reset",
        "m2b_attempt_topic": f"/{ns}/m2b/attempt_number",
        "m2b_pendulum_topic": f"/{ns}/pendulum_swing_state",
        "m2b_latch_stable_topic": f"/{ns}/m2b/latch_stable",
        "m2b_latch_stability_topic": f"/{ns}/m2b/latch_stability",
        "m2b_joint_truth_timeout_s": 0.35,
        "m2b_bootstrap_detach_dwell_s": 0.15,
        "m2b_bootstrap_settle_s": 0.50,
        "m2b_bootstrap_timeout_s": 300.0,
        "m2b_settle_speed_mps": 0.03,
        "m2b_settle_swing_deg": 3.0,
        "m2b_settle_position_tolerance_m": 0.05,
        "m2b_settle_dwell_s": 0.75,
        "m2b_approach_evidence_loss_abort_s": 0.75,
        "m2b_capture_height_m": 0.30,
        "m2b_tether_anchor_body_x": 0.0,
        "m2b_tether_anchor_body_y": 0.0,
        "m2b_tether_anchor_body_z": -0.04,
        "m2b_anchor_to_contact_length_m": 0.475,
        "m2b_high_low_split_m": 0.08,
        "m2b_high_xy_tolerance_m": 0.05,
        "m2b_low_xy_tolerance_m": 0.025,
        "m2b_gross_xy_abort_m": 0.10,
        "m2b_alignment_pause_timeout_s": 2.0,
        "m2b_descent_speed_mps": 0.04,
        "m2b_magnet_enable_height_m": 0.04,
        "m2b_magnet_enable_relative_speed_mps": 0.12,
        "m2b_capture_hold_normal_m": 0.010,
        "m2b_physical_latch_timeout_s": 0.75,
        "m2b_latch_settle_dwell_s": 0.50,
        "m2b_latch_settle_timeout_s": 1.50,
        "m2b_latch_settle_speed_mps": 0.08,
        "m2b_latch_settle_accel_mps2": 2.0,
        "m2b_latch_abort_speed_mps": 0.50,
        "m2b_latch_abort_accel_mps2": 8.0,
        "m2b_latch_abort_displacement_m": 0.05,
        "m2b_proof_timeout_s": 2.0,
        "m2b_proof_direction_radial": 1.0,
        "m2b_proof_direction_tangential": 0.0,
        "m2b_proof_direction_normal": 0.0,
        "m2b_proof_command_m": 0.025,
        "m2b_proof_speed_mps": 0.03,
        "m2b_proof_min_measured_excitation_m": 0.012,
        "m2b_attached_hold_angle_deg": 15.0,
        "m2b_attached_loss_grace_s": 0.50,
        "m2b_detach_verify_timeout_s": 1.50,
        "m2b_retreat_rise_m": 0.20,
        "m2b_max_attempts": 3,
        "takeoff_height": 0.50,
        "takeoff_speed": 0.15,
        "min_reference_z": 0.10,
        "pickup_z_tolerance": 0.10,
        "reference_nominal_speed": 0.20,
        "use_measured_magnet_tip": False,
        "use_static_pickup_object": True,
        "static_obstacle_count": 0,
        "drone_state_topic": f"/{ns}/motion_capture_state",
        "reference_topic": f"/{ns}/join_planner/reference",
        "marker_topic": f"/{ns}/join_planner/markers",
        "state_topic": f"/{ns}/join_planner/state",
        "phase_topic": f"/{ns}/join_planner/phase",
        "handoff_ready_topic": f"/{ns}/join_planner/handoff_ready",
        "obstacle_diagnostics_topic": f"/{ns}/join_planner/obstacle_diagnostics",
        "drone_command_topic": f"/{ns}/command",
        "magnet_command_topic": f"/{ns}/magnet/command",
        "object_attached_topic": f"/{ns}/magnet/object_attached",
        "magnet_tip_pose_topic": f"/{ns}/magnet_tip_pose",
        "external_landing_topic": f"/{ns}/join_planner/land_now",
        "publish_committed_trajectory": True,
        "committed_trajectory_topic": f"/{ns}/committed_trajectory",
        "committed_trajectory_horizon_s": 10.0,
        "enable_csv_logging": True,
        "log_directory": PathJoinSubstitution([log_root, ns, "join_planner"]),
        "log_run_label": "m2d",
    }


def _backend_params(drone_id: int, log_root, pre_authority_octopus_runtime_s):
    ns = f"drone_{drone_id}"
    peers = [f"drone_{j}" for j in DRONE_IDS if j != drone_id]
    return {
        "use_sim_time": True,
        "vehicle_id": ns,
        "frame_id": "map",
        "state_topic": f"/{ns}/motion_capture_state",
        "target_topic": f"/{ns}/dynamic_planner/transfer_target",
        "target_velocity_topic": f"/{ns}/dynamic_planner/transfer_target_velocity",
        "object_attached_topic": f"/{ns}/magnet/object_attached",
        "enable_topic": f"/{ns}/dynamic_planner/transfer_enable",
        "authority_topic": f"/{ns}/dynamic_planner/transfer_authority",
        "authority_ack_topic": f"/{ns}/dynamic_planner/transfer_authority_ack",
        "shadow_reference_topic": f"/{ns}/dynamic_planner/transfer_reference",
        "diagnostics_topic": f"/{ns}/dynamic_planner/transfer_status",
        "marker_topic": f"/{ns}/dynamic_planner/markers",
        "cooperative_scene_enabled": True,
        "cooperative_prediction_mode": "shared_trajectory",
        "cooperative_drone_state_topics": [f"/{peer}/motion_capture_state" for peer in peers],
        "cooperative_committed_trajectory_topics": [f"/{peer}/committed_trajectory" for peer in peers],
        "cooperative_vehicle_ids": peers,
        "cooperative_attached_tether_enabled": True,
        "cooperative_attached_plate_topics": [f"/{peer}/m2d/attached_plate_id" for peer in peers],
        "cooperative_ring_pose_topic": "/model/payload_model/pose",
        "cooperative_ring_pose_index": 1,
        "cooperative_ring_pose_timeout_s": 0.50,
        "cooperative_ring_plate_count": 12,
        "cooperative_ring_plate_pitch_diameter_m": 0.50,
        "cooperative_attached_tether_radius_m": 0.015,
        "cooperative_tether_anchor_x_m": 0.0,
        "cooperative_tether_anchor_y_m": 0.0,
        "cooperative_tether_anchor_z_m": -0.04,
        "moving_basket_scene_enabled": True,
        "moving_basket_committed_trajectory_topic": "/m2d/ring/committed_trajectory",
        "moving_basket_collision_mode": "segmented_ring",
        "moving_ring_segment_count": M2D_RING_SEGMENT_COUNT,
        "moving_ring_segment_center_radius_m": M2D_RING_SEGMENT_CENTER_RADIUS_M,
        "moving_ring_segment_tangential_length_m": M2D_RING_SEGMENT_TANGENTIAL_LENGTH_M,
        "moving_ring_segment_radial_width_m": M2D_RING_SEGMENT_RADIAL_WIDTH_M,
        "moving_ring_segment_height_m": M2D_RING_SEGMENT_HEIGHT_M,
        "moving_ring_segment_center_z_offset_m": M2D_RING_SEGMENT_CENTER_Z_OFFSET_M,
        "moving_ring_segment_padding_m": M2D_RING_SEGMENT_PADDING_M,
        # M2D-only handoff/liveness tuning. Keep the accepted prepared trajectory
        # shape when it still passes a fresh current-world collision certificate,
        # rather than unconditionally throwing it away and restarting from hover.
        "reuse_prepared_commit_on_authority": True,
        # The four-drone segmented-ring + cooperative scene made the inherited
        # 50 ms Octopus budget runtime-limited on essentially every drone-2 solve.
        # 100 ms remains below the 5 Hz (200 ms) planner period.
        "octopus_max_runtime_s": 0.10,
        "octopus_pre_authority_max_runtime_s": ParameterValue(
            pre_authority_octopus_runtime_s, value_type=float
        ),
        "static_scene_enabled": False,
        "require_scene_witness": False,
        "payload_collision_enabled": False,
        "require_payload_attached_for_enable": False,
        "allow_live_target_updates": True,
        "target_predictor_type": "committed_trajectory",
        "csv_path": PathJoinSubstitution([log_root, ns, "backend.csv"]),
        "replan_csv_path": PathJoinSubstitution([log_root, ns, "replans.csv"]),
    }


def generate_launch_description() -> LaunchDescription:
    drone_share = Path(get_package_share_directory("tejen_mission"))
    dynamic_share = Path(get_package_share_directory("tejen_dynamic_planner"))
    commissioned_backend_params = _load_commissioned_backend_params(dynamic_share)
    gui = LaunchConfiguration("gui")
    world_path = LaunchConfiguration("world_path")
    log_root = LaunchConfiguration("m2d_log_dir")
    pre_authority_octopus_runtime_s = LaunchConfiguration(
        "octopus_pre_authority_max_runtime_s"
    )
    target_attachment_count = LaunchConfiguration("target_attachment_count")
    operator_hold_after_goal = LaunchConfiguration("operator_hold_after_goal")

    actions = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("world_path"),
        DeclareLaunchArgument("m2d_log_dir", default_value="logs/m2_attachment/m2d"),
        DeclareLaunchArgument(
            "octopus_pre_authority_max_runtime_s", default_value="0.10"
        ),
        DeclareLaunchArgument("target_attachment_count", default_value="4"),
        DeclareLaunchArgument("operator_hold_after_goal", default_value="false"),
        # all four vehicles hold mission permission at once (testing; multi_drone_control)
        DeclareLaunchArgument("simultaneous", default_value="false"),
        ExecuteProcess(
            cmd=["gz", "sim", "-v", "2", world_path],
            name="m2d_gazebo_gui", output="screen", condition=IfCondition(gui),
            sigterm_timeout="3", sigkill_timeout="2",
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-v", "2", world_path],
            name="m2d_gazebo_headless", output="screen", condition=UnlessCondition(gui),
            sigterm_timeout="3", sigkill_timeout="2",
        ),
    ]

    actions.append(TimerAction(period=0.50, actions=[Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="m2d_clock_bridge",
        output="screen", arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )]))
    actions.append(TimerAction(period=0.75, actions=[
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="m2d_ring_pose_bridge",
            output="screen",
            arguments=["/model/payload_model/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V"],
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="m2d_joint_command_bridge",
            output="screen",
            parameters=[{"config_file": str(drone_share / "config" / "m2c_gz_joint_bridge.yaml")}],
        ),
    ]))

    platform_nodes = []
    truth_nodes = []
    attachment_nodes = []
    planner_nodes = []
    backend_nodes = []

    for drone_id in DRONE_IDS:
        ns = f"drone_{drone_id}"
        model = f"x3_{drone_id}"
        pose_topic = f"/model/{model}/pose"
        motor_topic = f"/{ns}/gazebo/command/motor_speed"

        platform_nodes.extend([
            Node(
                package="ros_gz_bridge", executable="parameter_bridge",
                name=f"m2d_pose_motor_bridge_{drone_id}", output="screen",
                arguments=[
                    f"{pose_topic}@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V",
                    f"{motor_topic}@actuator_msgs/msg/Actuators]ignition.msgs.Actuators",
                ],
            ),
            Node(
                package="simulation_communication", executable="tejen_motion_capture_emulator",
                namespace=ns, name="motion_capture_emulator", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "target_object_id": 7,
                    "pose_topic": pose_topic,
                    "motion_capture_state_topic": "motion_capture_state",
                    "rviz_pose_topic": "rviz_pose",
                    "enable_orientation_bias": False,
                }],
            ),
            Node(
                package="simulation_communication", executable="tejen_betaflight_communication",
                namespace=ns, name="betaflight_communication", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "pose_topic": pose_topic,
                    "pose_index": 7,
                    "elrs_command_topic": "ELRSCommand",
                    "motor_command_topic": motor_topic,
                    "publish_telemetry": True,
                    "telemetry_topic": "telemetry",
                    "rates_d_val": 100.0,
                    "rates_f_val": 100.0,
                    "rates_g_val": 0.0,
                    # multi_drone_control rate-loop I-term (T0011); 0 = the old P-only loop.
                    # Falls back to SIM_RATE_KI so that one env var reverts every sim bridge.
                    "rate_ki": float(os.environ.get(
                        "M2_RATE_KI", os.environ.get("SIM_RATE_KI", "5.0"))),
                }],
            ),
            Node(
                package="tejen_mission", executable="pendulum_state_publisher",
                namespace=ns, name="pendulum_state_publisher", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "x3_pose_topic": pose_topic,
                    "magnet_index": 0,
                    "attachment_index": 5,
                    "drone_index": 7,
                    "x3_link_poses_are_relative": True,
                    "use_drone_as_attachment_anchor": True,
                    "attachment_offset_x": 0.0,
                    "attachment_offset_y": 0.0,
                    "attachment_offset_z": -0.05,
                    "object_pose_topic": "/model/payload_model/pose",
                    "publish_payload_world_state": False,
                    "pendulum_state_topic": "pendulum_swing_state",
                    "magnet_tip_pose_topic": "magnet_tip_pose",
                    "publish_rate_hz": 120.0,
                }],
            ),
        ])

        truth_nodes.append(Node(
            package="tejen_mission", executable="m2a_gazebo_joint_truth_bridge",
            namespace=ns, name="m2d_joint_truth_bridge", output="screen",
            parameters=[{
                "use_sim_time": True,
                "gz_topic": f"/{ns}/magnet/joint_detached_truth",
                "ros_topic": f"/{ns}/magnet/joint_detached_truth",
                "status_topic": "magnet/joint_truth_bridge_state",
                "republish_period_s": 0.10,
            }],
        ))

        attachment_nodes.extend([
            Node(
                package="tejen_mission", executable="m2a_physical_capture_manager",
                namespace=ns, name="m2d_physical_capture_manager", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "pose_source": "pose_array",
                    "x3_pose_topic": pose_topic,
                    "magnet_index": 0,
                    "drone_index": 7,
                    "x3_link_poses_are_relative": True,
                    "ring_pose_source": "pose_array",
                    "ring_pose_topic": "/model/payload_model/pose",
                    "ring_pose_index": 1,
                    "assigned_plate_topic": f"/{ns}/m2d/assigned_plate_id",
                    "force_initial_detach": False,
                    "raw_detach_request_topic": f"/{ns}/m2b/sim/raw_detach_request",
                    "magnet_command_topic": f"/{ns}/magnet/command",
                    "contact_observation_model": "sphere_center",
                    "sphere_radius_m": 0.025,
                    "physical_capture_xy_m": 0.020,
                    "physical_capture_normal_m": 0.010,
                    "physical_capture_speed_mps": 0.12,
                    "physical_capture_dwell_s": 0.10,
                    "command_backend": "ros_bridge",
                    "ros_attach_command_topic": f"/{ns}/magnet/attach",
                    "ros_detach_command_topic": f"/{ns}/magnet/detach",
                    "attach_commanded_topic": f"/{ns}/sim/attach_commanded",
                    "capture_state_topic": f"/{ns}/sim/capture_state",
                }],
            ),
            Node(
                package="tejen_mission", executable="m2_attachment_observer",
                namespace=ns, name="m2d_attachment_observer", output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "pose_source": "pose_array",
                    "x3_pose_topic": pose_topic,
                    "magnet_index": 0,
                    "drone_index": 7,
                    "x3_link_poses_are_relative": True,
                    "ring_pose_source": "pose_array",
                    "ring_pose_topic": "/model/payload_model/pose",
                    "ring_pose_index": 1,
                    "assigned_plate_topic": f"/{ns}/m2d/assigned_plate_id",
                    "magnet_command_topic": f"/{ns}/magnet/command",
                    "proof_requested_topic": f"/{ns}/attachment/proof_requested",
                    "reset_topic": f"/{ns}/attachment/reset",
                    "diagnostics_topic": f"/{ns}/attachment/diagnostics",
                    "state_topic": f"/{ns}/attachment/state",
                    "confirmed_topic": f"/{ns}/attachment/confirmed",
                    "probe_phase_topic": f"/{ns}/attachment/probe_phase",
                    "joint_truth_topic": f"/{ns}/magnet/joint_detached_truth",
                    "contact_observation_model": "sphere_center",
                    "sphere_radius_m": 0.025,
                    "proof_direction_radial": 1.0,
                    "proof_direction_tangential": 0.0,
                    "proof_direction_normal": 0.0,
                    "proof_speed_mps": 0.15,
                    "proof_min_excitation_m": 0.012,
                    "log_dir": PathJoinSubstitution([log_root, ns, "attachment"]),
                    "metadata_filename": "attachment_metadata.json",
                    "vehicle_pose_semantics": f"real X3/base_link pose during M2D sequential attachment ({ns})",
                }],
            ),
        ])

        planner_nodes.append(Node(
            package="tejen_mission", executable="online_join_planner",
            namespace=ns, name="online_join_planner", output="screen",
            parameters=[_m2b_params(drone_id, log_root)],
        ))
        backend_nodes.append(Node(
            package="tejen_dynamic_planner", executable="dynamic_planner_transfer_backend",
            namespace=ns, name="dynamic_planner_transfer_backend", output="screen",
            parameters=[
                deepcopy(commissioned_backend_params),
                _backend_params(
                    drone_id, log_root, pre_authority_octopus_runtime_s
                ),
            ],
        ))

    shared_nodes = [
        Node(
            package="tejen_mission", executable="m2c_fleet_manager",
            name="m2c_fleet_manager", output="screen",
            parameters=[{
                "use_sim_time": True,
                "ring_pose_topic": "/model/payload_model/pose",
                "ring_pose_index": 1,
                "vehicle_state_timeout_s": 0.25,
                "ring_state_timeout_s": 0.25,
                "commitment_timeout_s": 0.50,
                "joint_truth_timeout_s": 0.25,
                "ground_settle_dwell_s": 1.0,
                "mocap_rigid_body_ids": [100, 101, 102, 103],
            }],
        ),
        Node(
            package="tejen_mission", executable="m2d_ring_commitment",
            name="m2d_ring_commitment", output="screen",
            parameters=[{
                "use_sim_time": True,
                "ring_pose_topic": "/model/payload_model/pose",
                "ring_pose_index": 1,
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
                "use_sim_time": True,
                "m2c_assignment_topic": "/m2c/assignment",
                "m2c_ready_topic": "/m2c/ready",
                "attachment_evidence_timeout_s": 0.75,
                "target_attachment_count": ParameterValue(
                    target_attachment_count, value_type=int
                ),
                "simultaneous": ParameterValue(
                    LaunchConfiguration("simultaneous"), value_type=bool),
                "operator_hold_after_goal": ParameterValue(
                    operator_hold_after_goal, value_type=bool
                ),
                "operator_land_request_topic": "/m2d/operator/land_request",
                "status_topic": "/m2d/status",
                "pass_topic": "/m2d/pass",
                "publish_rate_hz": 10.0,
            }],
        ),
    ]

    actions.extend([
        TimerAction(period=1.00, actions=platform_nodes),
        TimerAction(period=1.20, actions=truth_nodes),
        TimerAction(period=1.35, actions=attachment_nodes),
        TimerAction(period=1.55, actions=planner_nodes),
        TimerAction(period=1.80, actions=backend_nodes),
        TimerAction(period=2.00, actions=shared_nodes),
    ])
    return LaunchDescription(actions)
