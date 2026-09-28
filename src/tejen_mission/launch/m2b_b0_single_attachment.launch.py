"""Launch M2B B0: grounded real X3, bootstrap detach, takeoff and hover.

The world intentionally starts paused. The supervised runner advances Gazebo just
enough for the DetachableJoint bootstrap transition, then unpauses after raw joint
truth reports detached. Mission authority remains in online_join_planner.
"""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from simulation_communication.imu_world import launch_guard


def _workspace_root() -> Path:
    return Path(get_package_prefix("tejen_mission")).resolve().parents[1]


def generate_launch_description() -> LaunchDescription:
    root = _workspace_root()
    simulation_share = Path(get_package_share_directory("simulation_communication"))
    visualisation_share = Path(get_package_share_directory("drone_visualisation"))

    gui = LaunchConfiguration("gui")
    assigned_plate_id = LaunchConfiguration("assigned_plate_id")
    takeoff_height = LaunchConfiguration("takeoff_height")
    m2b_log_dir = LaunchConfiguration("m2b_log_dir")
    world_path = LaunchConfiguration("world_path")

    gazebo_gui = ExecuteProcess(
        cmd=["gz", "sim", "-v", "2", world_path],
        name="m2b_b0_gazebo_gui",
        output="screen",
        condition=IfCondition(gui),
        sigterm_timeout="3",
        sigkill_timeout="2",
    )
    gazebo_headless = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-v", "2", world_path],
        name="m2b_b0_gazebo_headless",
        output="screen",
        condition=UnlessCondition(gui),
        sigterm_timeout="3",
        sigkill_timeout="2",
    )

    simulation_interfaces = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(simulation_share / "launch" / "tejen_betaflight_linear_simulation_launch.py")
        )
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(visualisation_share / "launch" / "view_frame.launch.py")
        ),
        condition=IfCondition(gui),
    )

    pendulum = Node(
        package="tejen_mission",
        executable="pendulum_state_publisher",
        name="pendulum_state_publisher",
        output="screen",
        parameters=[{
            "x3_pose_topic": "/model/x3/pose",
            "magnet_index": 0,
            "attachment_index": 5,
            "drone_index": 7,
            "x3_link_poses_are_relative": True,
            "use_drone_as_attachment_anchor": True,
            # Current modelLargeM2BallMagnet.sdf base_to_tether joint offset.
            "attachment_offset_x": 0.0,
            "attachment_offset_y": 0.0,
            "attachment_offset_z": -0.05,
            "publish_payload_world_state": False,
        }],
    )

    joint_truth = Node(
        package="tejen_mission",
        executable="m2a_gazebo_joint_truth_bridge",
        name="m2b_gazebo_joint_truth_bridge",
        output="screen",
        parameters=[{
            "gz_topic": "/payload/detachable_joint_state",
            "ros_topic": "/m2a/sim/joint_detached_truth",
            "republish_period_s": 0.10,
        }],
    )

    physical_capture = Node(
        package="tejen_mission",
        executable="m2a_physical_capture_manager",
        name="m2b_physical_capture_manager",
        output="screen",
        parameters=[{
            "pose_source": "pose_array",
            "x3_pose_topic": "/model/x3/pose",
            "magnet_index": 0,
            "drone_index": 7,
            "x3_link_poses_are_relative": True,
            "assigned_plate_id": ParameterValue(assigned_plate_id, value_type=int),
            "fixed_ring_x": 0.0,
            "fixed_ring_y": 0.0,
            "fixed_ring_z": 0.035,
            "force_initial_detach": False,
            "raw_detach_request_topic": "/m2b/sim/raw_detach_request",
            # M2 commissioning capture envelope - intentionally much tighter than M1.
            "physical_capture_xy_m": 0.020,
            "physical_capture_normal_m": 0.020,
            "physical_capture_speed_mps": 0.12,
            "physical_capture_dwell_s": 0.10,
        }],
    )

    telemetry = Node(
        package="tejen_mission",
        executable="m2b_b0_telemetry",
        name="m2b_b0_telemetry",
        output="screen",
        parameters=[{
            "log_dir": m2b_log_dir,
            "sample_rate_hz": 20.0,
            "assigned_plate_id": ParameterValue(assigned_plate_id, value_type=int),
        }],
    )

    join_planner = Node(
        package="tejen_mission",
        executable="online_join_planner",
        name="online_join_planner",
        output="screen",
        parameters=[{
            "vehicle_id": "drone_0",
            "assigned_attachment_id": "attachment_0",
            "mission_mode": "m2b",
            "m2b_commissioning_stage": "b0",
            "m2b_assigned_plate_id": ParameterValue(assigned_plate_id, value_type=int),
            "m2b_joint_truth_topic": "/m2a/sim/joint_detached_truth",
            "m2b_raw_detach_request_topic": "/m2b/sim/raw_detach_request",
            "m2b_arm_permission_topic": "/join_planner/arm_permission",
            "m2b_mpc_mode_topic": "/join_planner/mpc_mode",
            "m2b_arming_state_topic": "drone_arming_state_feedback",
            "m2b_joint_truth_timeout_s": 0.35,
            "m2b_bootstrap_detach_dwell_s": 0.15,
            "m2b_bootstrap_settle_s": 0.50,
            # Commissioning startup includes paused pose initialization before the
            # first useful DetachableJoint step. Arm remains hard-blocked throughout.
            "m2b_bootstrap_timeout_s": 8.0,
            "takeoff_height": ParameterValue(takeoff_height, value_type=float),
            "takeoff_speed": 0.15,
            # Ground-start references must not inherit the legacy 0.60 m clamp.
            "min_reference_z": 0.10,
            "pickup_z_tolerance": 0.10,
            "use_measured_magnet_tip": False,
            "use_static_pickup_object": True,
            "static_obstacle_count": 0,
            "log_directory": "logs/join_planner",
            "log_run_label": "m2b_b0",
        }],
    )

    controller = Node(
        package="tejen_mpc",
        executable="main",
        name="tejen_mpc",
        output="screen",
        parameters=[{
            "use_external_reference": True,
            "external_reference_topic": "/join_planner/reference",
            "enable_thrust_ratio_ukf": True,
            "require_external_arm_permission": True,
            "external_arm_permission_topic": "/join_planner/arm_permission",
            "payload_mpc_mode_topic": "/join_planner/mpc_mode",
        }],
    )

    # Gazebo starts first and remains paused. Bridges/helpers then come up before
    # the mission executive; the MPC starts last because ACADOS construction is
    # relatively expensive and all upstream interfaces should already exist.
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("assigned_plate_id", default_value="0"),
        DeclareLaunchArgument("takeoff_height", default_value="0.50"),
        DeclareLaunchArgument("m2b_log_dir", default_value="logs/m2_attachment"),
        DeclareLaunchArgument("world_path",
            default_value=str(root / "simulation_assets" / "tejen" / "world_m2b_single_attachment.sdf"),
        ),
        # rate_source imu (the sim bridge default): refuse a world without the Imu system
        launch_guard(world_config="world_path"),
        gazebo_gui,
        gazebo_headless,
        TimerAction(period=0.75, actions=[simulation_interfaces]),
        TimerAction(period=1.00, actions=[pendulum, joint_truth, physical_capture, telemetry]),
        TimerAction(period=1.25, actions=[join_planner]),
        TimerAction(period=1.50, actions=[controller]),
        TimerAction(period=1.75, actions=[rviz]),
    ])
