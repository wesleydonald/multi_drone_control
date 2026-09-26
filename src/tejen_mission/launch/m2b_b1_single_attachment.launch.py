"""Launch M2B B1/B2 single-drone attachment commissioning.

B1 preserves the commissioned local attachment path. B2 optionally starts the
existing C++ transfer backend for collision-avoiding transit to the exact B1
capture hover, then hands back to the unchanged B1 settle/descent/proof logic.
"""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_root() -> Path:
    return Path(get_package_prefix("tejen_mission")).resolve().parents[1]


def generate_launch_description() -> LaunchDescription:
    root = _workspace_root()
    drone_share = Path(get_package_share_directory("tejen_mission"))
    simulation_share = Path(get_package_share_directory("simulation_communication"))
    visualisation_share = Path(get_package_share_directory("drone_visualisation"))
    dynamic_share = Path(get_package_share_directory("tejen_dynamic_planner"))

    gui = LaunchConfiguration("gui")
    assigned_plate_id = LaunchConfiguration("assigned_plate_id")
    takeoff_height = LaunchConfiguration("takeoff_height")
    m2b_log_dir = LaunchConfiguration("m2b_log_dir")
    world_path = LaunchConfiguration("world_path")
    attachment_enabled = LaunchConfiguration("attachment_enabled")
    proof_enabled = LaunchConfiguration("proof_enabled")
    max_attempts = LaunchConfiguration("max_attempts")
    proof_direction_radial = LaunchConfiguration("proof_direction_radial")
    proof_direction_tangential = LaunchConfiguration("proof_direction_tangential")
    proof_direction_normal = LaunchConfiguration("proof_direction_normal")
    proof_command_m = LaunchConfiguration("proof_command_m")
    proof_command_speed_mps = LaunchConfiguration("proof_command_speed_mps")
    proof_detector_speed_limit_mps = LaunchConfiguration("proof_detector_speed_limit_mps")
    proof_min_excitation_m = LaunchConfiguration("proof_min_excitation_m")
    attached_hold_angle_deg = LaunchConfiguration("attached_hold_angle_deg")
    contact_observation_model = LaunchConfiguration("contact_observation_model")
    magnet_sphere_radius_m = LaunchConfiguration("magnet_sphere_radius_m")
    magnet_capture_gap_m = LaunchConfiguration("magnet_capture_gap_m")
    latch_settle_dwell_s = LaunchConfiguration("latch_settle_dwell_s")
    latch_settle_timeout_s = LaunchConfiguration("latch_settle_timeout_s")
    latch_settle_speed_mps = LaunchConfiguration("latch_settle_speed_mps")
    latch_settle_accel_mps2 = LaunchConfiguration("latch_settle_accel_mps2")
    latch_abort_speed_mps = LaunchConfiguration("latch_abort_speed_mps")
    latch_abort_accel_mps2 = LaunchConfiguration("latch_abort_accel_mps2")
    latch_abort_displacement_m = LaunchConfiguration("latch_abort_displacement_m")
    commissioning_stage = LaunchConfiguration("commissioning_stage")
    b2_cpp_transit_enabled = LaunchConfiguration("b2_cpp_transit_enabled")
    b2_static_obstacle_enabled = LaunchConfiguration("b2_static_obstacle_enabled")

    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-v", "2", world_path],
        name="m2b_b1_gazebo_gui", output="screen", condition=IfCondition(gui),
        sigterm_timeout="3", sigkill_timeout="2",
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-v", "2", world_path],
        name="m2b_b1_gazebo_headless", output="screen", condition=UnlessCondition(gui),
        sigterm_timeout="3", sigkill_timeout="2",
    )

    simulation_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(simulation_share / "launch" / "tejen_betaflight_linear_simulation_launch.py"))
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(visualisation_share / "launch" / "view_frame.launch.py")),
        condition=IfCondition(gui),
    )

    joint_command_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", name="m2b_joint_command_bridge",
        output="screen",
        parameters=[{"config_file": str(drone_share / "config" / "m2b_gz_joint_bridge.yaml")}],
    )

    pendulum = Node(
        package="tejen_mission", executable="pendulum_state_publisher", name="pendulum_state_publisher",
        output="screen",
        parameters=[{
            "x3_pose_topic": "/model/x3/pose", "magnet_index": 0, "attachment_index": 5, "drone_index": 7,
            "x3_link_poses_are_relative": True, "use_drone_as_attachment_anchor": True,
            "attachment_offset_x": 0.0, "attachment_offset_y": 0.0, "attachment_offset_z": -0.05,
            "publish_payload_world_state": False,
        }],
    )
    joint_truth = Node(
        package="tejen_mission", executable="m2a_gazebo_joint_truth_bridge", name="m2b_gazebo_joint_truth_bridge",
        output="screen",
        parameters=[{"gz_topic": "/payload/detachable_joint_state", "ros_topic": "/m2a/sim/joint_detached_truth", "republish_period_s": 0.10}],
    )
    physical_capture = Node(
        package="tejen_mission", executable="m2a_physical_capture_manager", name="m2b_physical_capture_manager",
        output="screen",
        parameters=[{
            "pose_source": "pose_array", "x3_pose_topic": "/model/x3/pose", "magnet_index": 0, "drone_index": 7,
            "x3_link_poses_are_relative": True,
            "assigned_plate_id": ParameterValue(assigned_plate_id, value_type=int),
            "fixed_ring_x": 0.0, "fixed_ring_y": 0.0, "fixed_ring_z": 0.035,
            "force_initial_detach": False, "raw_detach_request_topic": "/m2b/sim/raw_detach_request",
            "contact_observation_model": contact_observation_model,
            "sphere_radius_m": ParameterValue(magnet_sphere_radius_m, value_type=float),
            "physical_capture_xy_m": 0.020,
            "physical_capture_normal_m": ParameterValue(magnet_capture_gap_m, value_type=float),
            "physical_capture_speed_mps": 0.12, "physical_capture_dwell_s": 0.10,
            "command_backend": "ros_bridge",
            "ros_attach_command_topic": "/m2b/gz/attach", "ros_detach_command_topic": "/m2b/gz/detach",
        }],
    )
    observer = Node(
        package="tejen_mission", executable="m2_attachment_observer", name="m2b_attachment_observer",
        output="screen",
        parameters=[{
            "pose_source": "pose_array", "x3_pose_topic": "/model/x3/pose", "magnet_index": 0, "drone_index": 7,
            "x3_link_poses_are_relative": True,
            "ring_pose_source": "fixed", "fixed_ring_x": 0.0, "fixed_ring_y": 0.0, "fixed_ring_z": 0.035,
            "assigned_plate_id": ParameterValue(assigned_plate_id, value_type=int),
            "magnet_command_topic": "/magnet/command",
            "proof_requested_topic": "/m2a/attachment/proof_requested",
            "reset_topic": "/m2a/attachment/reset",
            "diagnostics_topic": "/m2a/attachment/diagnostics",
            "joint_truth_topic": "/m2a/sim/joint_detached_truth",
            "contact_observation_model": contact_observation_model,
            "sphere_radius_m": ParameterValue(magnet_sphere_radius_m, value_type=float),
            "proof_direction_radial": ParameterValue(proof_direction_radial, value_type=float),
            "proof_direction_tangential": ParameterValue(proof_direction_tangential, value_type=float),
            "proof_direction_normal": ParameterValue(proof_direction_normal, value_type=float),
            "proof_speed_mps": ParameterValue(proof_detector_speed_limit_mps, value_type=float),
            "proof_min_excitation_m": ParameterValue(proof_min_excitation_m, value_type=float),
            "log_dir": m2b_log_dir, "metadata_filename": "attachment_metadata.json",
            "vehicle_pose_semantics": "real X3/base_link pose during M2B B1 autonomous attachment",
        }],
    )
    telemetry = Node(
        package="tejen_mission", executable="m2b_b1_telemetry", name="m2b_b1_telemetry", output="screen",
        parameters=[{
            "log_dir": m2b_log_dir, "sample_rate_hz": 20.0,
            "assigned_plate_id": ParameterValue(assigned_plate_id, value_type=int),
            "contact_observation_model": contact_observation_model,
            "sphere_radius_m": ParameterValue(magnet_sphere_radius_m, value_type=float),
            "magnet_capture_gap_m": ParameterValue(magnet_capture_gap_m, value_type=float),
            "latch_settle_dwell_s": ParameterValue(latch_settle_dwell_s, value_type=float),
            "latch_settle_timeout_s": ParameterValue(latch_settle_timeout_s, value_type=float),
            "latch_settle_speed_mps": ParameterValue(latch_settle_speed_mps, value_type=float),
            "latch_settle_accel_mps2": ParameterValue(latch_settle_accel_mps2, value_type=float),
            "latch_abort_speed_mps": ParameterValue(latch_abort_speed_mps, value_type=float),
            "latch_abort_accel_mps2": ParameterValue(latch_abort_accel_mps2, value_type=float),
            "latch_abort_displacement_m": ParameterValue(latch_abort_displacement_m, value_type=float),
            "proof_direction_radial": ParameterValue(proof_direction_radial, value_type=float),
            "proof_direction_tangential": ParameterValue(proof_direction_tangential, value_type=float),
            "proof_direction_normal": ParameterValue(proof_direction_normal, value_type=float),
            "proof_command_m": ParameterValue(proof_command_m, value_type=float),
            "proof_command_speed_mps": ParameterValue(proof_command_speed_mps, value_type=float),
            "proof_detector_speed_limit_mps": ParameterValue(proof_detector_speed_limit_mps, value_type=float),
            "proof_min_excitation_m": ParameterValue(proof_min_excitation_m, value_type=float),
            "attached_hold_angle_deg": ParameterValue(attached_hold_angle_deg, value_type=float),
        }],
    )
    join_planner = Node(
        package="tejen_mission", executable="online_join_planner", name="online_join_planner", output="screen",
        parameters=[{
            "vehicle_id": "drone_0", "assigned_attachment_id": "attachment_0", "mission_mode": "m2b",
            "m2b_commissioning_stage": commissioning_stage,
            "c1f1_shadow_transfer_enabled": ParameterValue(b2_cpp_transit_enabled, value_type=bool),
            "c1f2_cpp_authority_enabled": ParameterValue(b2_cpp_transit_enabled, value_type=bool),
            "m2b_b1_attachment_enabled": ParameterValue(attachment_enabled, value_type=bool),
            "m2b_b1_proof_enabled": ParameterValue(proof_enabled, value_type=bool),
            "m2b_assigned_plate_id": ParameterValue(assigned_plate_id, value_type=int),
            "m2b_joint_truth_topic": "/m2a/sim/joint_detached_truth",
            "m2b_bootstrap_release_required": True,
            "m2b_bootstrap_release_topic": "/m2b/bootstrap/release",
            "m2b_raw_detach_request_topic": "/m2b/sim/raw_detach_request",
            "m2b_arm_permission_topic": "/join_planner/arm_permission", "m2b_mpc_mode_topic": "/join_planner/mpc_mode",
            "m2b_arming_state_topic": "drone_arming_state_feedback",
            "m2b_attachment_diagnostics_topic": "/m2a/attachment/diagnostics",
            "m2b_proof_requested_topic": "/m2a/attachment/proof_requested", "m2b_detector_reset_topic": "/m2a/attachment/reset",
            "m2b_attempt_topic": "/m2b/attempt_number", "m2b_pendulum_topic": "/pendulum_swing_state",
            "m2b_latch_stable_topic": "/m2b/latch_stable", "m2b_latch_stability_topic": "/m2b/latch_stability",
            "m2b_joint_truth_timeout_s": 0.35, "m2b_bootstrap_detach_dwell_s": 0.15,
            "m2b_bootstrap_settle_s": 0.50, "m2b_bootstrap_timeout_s": 8.0,
            # Based on the commissioned B0 evidence: first-hover transient peaked near
            # 0.24 m/s / 24 deg, while 5-10 s was below ~0.017 m/s / 1.3 deg.
            "m2b_settle_speed_mps": 0.03, "m2b_settle_swing_deg": 3.0,
            "m2b_settle_position_tolerance_m": 0.05, "m2b_settle_dwell_s": 0.75,
            "m2b_approach_evidence_loss_abort_s": 0.75,
            "m2b_capture_height_m": 0.30,
            "m2b_tether_anchor_body_x": 0.0, "m2b_tether_anchor_body_y": 0.0, "m2b_tether_anchor_body_z": -0.04,
            "m2b_anchor_to_contact_length_m": 0.475,
            "m2b_high_low_split_m": 0.08, "m2b_high_xy_tolerance_m": 0.05, "m2b_low_xy_tolerance_m": 0.025,
            "m2b_gross_xy_abort_m": 0.10, "m2b_alignment_pause_timeout_s": 2.0,
            "m2b_descent_speed_mps": 0.04, "m2b_magnet_enable_height_m": 0.04,
            "m2b_magnet_enable_relative_speed_mps": 0.12,
            "m2b_capture_hold_normal_m": ParameterValue(magnet_capture_gap_m, value_type=float),
            "m2b_physical_latch_timeout_s": 0.75,
            "m2b_latch_settle_dwell_s": ParameterValue(latch_settle_dwell_s, value_type=float),
            "m2b_latch_settle_timeout_s": ParameterValue(latch_settle_timeout_s, value_type=float),
            "m2b_latch_settle_speed_mps": ParameterValue(latch_settle_speed_mps, value_type=float),
            "m2b_latch_settle_accel_mps2": ParameterValue(latch_settle_accel_mps2, value_type=float),
            "m2b_latch_abort_speed_mps": ParameterValue(latch_abort_speed_mps, value_type=float),
            "m2b_latch_abort_accel_mps2": ParameterValue(latch_abort_accel_mps2, value_type=float),
            "m2b_latch_abort_displacement_m": ParameterValue(latch_abort_displacement_m, value_type=float),
            "m2b_proof_timeout_s": 2.0,
            "m2b_proof_direction_radial": ParameterValue(proof_direction_radial, value_type=float),
            "m2b_proof_direction_tangential": ParameterValue(proof_direction_tangential, value_type=float),
            "m2b_proof_direction_normal": ParameterValue(proof_direction_normal, value_type=float),
            "m2b_proof_command_m": ParameterValue(proof_command_m, value_type=float),
            "m2b_proof_speed_mps": ParameterValue(proof_command_speed_mps, value_type=float),
            "m2b_proof_min_measured_excitation_m": ParameterValue(proof_min_excitation_m, value_type=float),
            "m2b_attached_hold_angle_deg": ParameterValue(attached_hold_angle_deg, value_type=float),
            "m2b_attached_loss_grace_s": 0.50,
            "m2b_detach_verify_timeout_s": 1.50, "m2b_retreat_rise_m": 0.20, "m2b_max_attempts": ParameterValue(max_attempts, value_type=int),
            "takeoff_height": ParameterValue(takeoff_height, value_type=float), "takeoff_speed": 0.15,
            "min_reference_z": 0.10, "pickup_z_tolerance": 0.10,
            # B1 retains the local transfer; B2 replaces only the post-takeoff capture transit with C++ authority.
            "reference_nominal_speed": 0.20,
            "use_measured_magnet_tip": False, "use_static_pickup_object": True, "static_obstacle_count": 0,
            "log_directory": "logs/join_planner", "log_run_label": "m2b_b1",
        }],
    )
    # B2 uses a stationary authoritative ring-centre commitment so the existing
    # moving-basket collision path represents the real ring/net without adding a
    # second target/collision geometry implementation. The long horizon allows
    # deliberate operator ARM/TAKEOFF delays during commissioning.
    b2_ring_commitment = Node(
        package="tejen_mission",
        executable="fake_cooperative_transport_world",
        name="m2b_b2_stationary_ring_commitment",
        output="screen",
        condition=IfCondition(b2_cpp_transit_enabled),
        parameters=[{
            "trajectory_type": "stationary",
            "obstacle_scenario": "custom",
            "center_x": 0.0, "center_y": 0.0, "center_z": 0.035,
            "payload_pose_topic": "/m2b/b2/ring_pose_unused",
            "payload_twist_topic": "/m2b/b2/ring_twist_unused",
            "marker_topic": "/m2b/b2/ring_markers_unused",
            "payload_committed_trajectory_topic": "/fake_payload/committed_trajectory",
            "enable_fake_obstacles": False,
            "fake_drone_count": 0,
            "publish_shared_trajectories": True,
            "shared_trajectory_duration_s": 900.0,
            "shared_trajectory_control_interval_s": 15.0,
        }],
    )

    b2_dynamic_planner = Node(
        package="tejen_dynamic_planner",
        executable="dynamic_planner_transfer_backend",
        name="dynamic_planner_transfer_backend",
        output="screen",
        condition=IfCondition(b2_cpp_transit_enabled),
        parameters=[
            str(dynamic_share / "config" / "c1f6_moving_rendezvous.yaml"),
            {
                "cooperative_scene_enabled": False,
                "payload_collision_enabled": False,
                "require_payload_attached_for_enable": False,
                "moving_basket_scene_enabled": True,
                "moving_basket_committed_trajectory_topic": "/fake_payload/committed_trajectory",
                "static_scene_enabled": ParameterValue(b2_static_obstacle_enabled, value_type=bool),
                "require_scene_witness": False,
                "obstacle_name": "m2b_b2_transit_obstacle",
                "obstacle_center_x_m": 0.68,
                "obstacle_center_y_m": 0.25,
                "obstacle_center_z_m": 1.0,
                "obstacle_half_x_m": 0.07,
                "obstacle_half_y_m": 0.12,
                "obstacle_half_z_m": 0.10,
                "obstacle_yaw_rad": 0.0,
                "csv_path": PathJoinSubstitution([m2b_log_dir, "backend.csv"]),
                "replan_csv_path": PathJoinSubstitution([m2b_log_dir, "replans.csv"]),
            },
        ],
    )

    controller = Node(
        package="tejen_mpc", executable="main", name="tejen_mpc", output="screen",
        parameters=[{
            "use_external_reference": True, "external_reference_topic": "/join_planner/reference",
            "enable_thrust_ratio_ukf": True,
            "thrust_ratio_estimator_backend": "full_model_kt_ukf",
            "thrust_ratio_estimator_rate_hz": 10.0,
            "enable_thrust_ratio_feedback": True,
            "thrust_ratio_feedback_min_updates": 15,
            "thrust_ratio_feedback_max_std": 1.5,
            "thrust_ratio_feedback_rate_per_s": 0.5,
            "thrust_ratio_feedback_max_fractional_change": 0.25,
            "thrust_ratio_feedback_deadband": 0.20,
            "require_external_arm_permission": True,
            "external_arm_permission_topic": "/join_planner/arm_permission", "payload_mpc_mode_topic": "/join_planner/mpc_mode",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"), DeclareLaunchArgument("assigned_plate_id", default_value="0"),
        DeclareLaunchArgument("commissioning_stage", default_value="b1"),
        DeclareLaunchArgument("b2_cpp_transit_enabled", default_value="false"),
        DeclareLaunchArgument("b2_static_obstacle_enabled", default_value="false"),
        DeclareLaunchArgument("takeoff_height", default_value="0.50"), DeclareLaunchArgument("m2b_log_dir", default_value="logs/m2_attachment"),
        DeclareLaunchArgument("attachment_enabled", default_value="true"),
        DeclareLaunchArgument("proof_enabled", default_value="true"),
        DeclareLaunchArgument("max_attempts", default_value="3"),
        DeclareLaunchArgument("contact_observation_model", default_value="sphere_center"),
        DeclareLaunchArgument("magnet_sphere_radius_m", default_value="0.025"),
        DeclareLaunchArgument("magnet_capture_gap_m", default_value="0.010"),
        # Post-latch commissioning gate.  The settle envelope must be quiet for
        # 0.5 s; gross dynamics fail before any proof motion is commanded.
        DeclareLaunchArgument("latch_settle_dwell_s", default_value="0.50"),
        DeclareLaunchArgument("latch_settle_timeout_s", default_value="1.50"),
        DeclareLaunchArgument("latch_settle_speed_mps", default_value="0.08"),
        DeclareLaunchArgument("latch_settle_accel_mps2", default_value="2.0"),
        DeclareLaunchArgument("latch_abort_speed_mps", default_value="0.50"),
        DeclareLaunchArgument("latch_abort_accel_mps2", default_value="8.0"),
        DeclareLaunchArgument("latch_abort_displacement_m", default_value="0.05"),
        # Simulation-feasible proof defaults.  These are launch arguments rather
        # than hidden constants so the same mission can be retuned for cage/IRL
        # validation without changing state-machine code.
        DeclareLaunchArgument("proof_direction_radial", default_value="1.0"),
        DeclareLaunchArgument("proof_direction_tangential", default_value="0.0"),
        DeclareLaunchArgument("proof_direction_normal", default_value="0.0"),
        DeclareLaunchArgument("proof_command_m", default_value="0.025"),
        DeclareLaunchArgument("proof_command_speed_mps", default_value="0.03"),
        DeclareLaunchArgument("proof_detector_speed_limit_mps", default_value="0.15"),
        DeclareLaunchArgument("proof_min_excitation_m", default_value="0.012"),
        DeclareLaunchArgument("attached_hold_angle_deg", default_value="15.0"),
        DeclareLaunchArgument("world_path", default_value=str(root / "simulation_assets" / "tejen" / "world_m2b_single_attachment.sdf")),
        gazebo_gui, gazebo_headless,
        TimerAction(period=0.75, actions=[simulation_interfaces, joint_command_bridge]),
        TimerAction(period=1.00, actions=[pendulum, joint_truth, observer, telemetry]),
        TimerAction(period=1.20, actions=[physical_capture]),
        TimerAction(period=1.25, actions=[b2_ring_commitment, b2_dynamic_planner]),
        TimerAction(period=1.45, actions=[join_planner]),
        TimerAction(period=1.70, actions=[controller]),
        TimerAction(period=1.95, actions=[rviz]),
    ])
