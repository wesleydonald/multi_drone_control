"""Ground-only M2C commissioning of four real X3 vehicle stacks.

The launch intentionally contains no flight sequencing.  It instantiates four
identity-separated simulation/platform/mission stacks, a shared fleet manager,
and the four startup DetachableJoint truth channels.  M2D owns motion.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


M2C_DRONE_IDS = (0, 1, 2, 3)
M2C_VEHICLE_NAMES = ("drone_0", "drone_1", "drone_2", "drone_3")
M2C_RING_YAW_DEG = 20.0
def generate_launch_description() -> LaunchDescription:
    drone_share = Path(get_package_share_directory("tejen_mission"))

    gui = LaunchConfiguration("gui")
    world_path = LaunchConfiguration("world_path")

    actions = [
        DeclareLaunchArgument("gui", default_value="true"),
        # M2C worlds are generated per commissioning run. Requiring this path avoids
        # a manual launch silently falling back to the historical single-X3 B1 world.
        DeclareLaunchArgument("world_path"),
        ExecuteProcess(
            cmd=["gz", "sim", "-v", "2", world_path],
            name="m2c_gazebo_gui",
            output="screen",
            condition=IfCondition(gui),
            sigterm_timeout="3",
            sigkill_timeout="2",
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-v", "2", world_path],
            name="m2c_gazebo_headless",
            output="screen",
            condition=UnlessCondition(gui),
            sigterm_timeout="3",
            sigkill_timeout="2",
        ),
    ]

    # P3 timing contract: bridge Gazebo simulation time once. All M2C
    # simulation ROS nodes follow /clock; only external host/process supervision
    # and computation profiling remain wall-time.
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="m2c_clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )
    actions.append(TimerAction(period=0.50, actions=[clock_bridge]))

    # Shared ring pose plus the eight attach/detach command bridges. Keep the
    # command bridge config separate from argument-mode bridging because Humble
    # parameter_bridge has stricter parsing when both forms are mixed.
    ring_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="m2c_ring_pose_bridge",
        output="screen",
        arguments=["/model/payload_model/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V"],
    )
    joint_command_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="m2c_joint_command_bridge",
        output="screen",
        parameters=[{"config_file": str(drone_share / "config" / "m2c_gz_joint_bridge.yaml")}],
    )
    actions.append(TimerAction(period=0.75, actions=[ring_pose_bridge, joint_command_bridge]))

    platform_actions = []
    mission_actions = []
    truth_actions = []
    for drone_id in M2C_DRONE_IDS:
        ns = f"drone_{drone_id}"
        model = f"x3_{drone_id}"
        vehicle = f"drone_{drone_id}"
        pose_topic = f"/model/{model}/pose"
        reference_topic = f"/{ns}/join_planner/reference"
        committed_topic = f"/{ns}/committed_trajectory"
        motor_topic = f"/{ns}/gazebo/command/motor_speed"

        platform_actions.extend([
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name=f"m2c_pose_motor_bridge_{drone_id}",
                output="screen",
                arguments=[
                    f"{pose_topic}@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V",
                    f"{motor_topic}@actuator_msgs/msg/Actuators]ignition.msgs.Actuators",
                ],
            ),
            Node(
                package="simulation_communication",
                executable="tejen_motion_capture_emulator",
                namespace=ns,
                name="motion_capture_emulator",
                output="screen",
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
                package="simulation_communication",
                executable="tejen_betaflight_communication",
                namespace=ns,
                name="betaflight_communication",
                output="screen",
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
                }],
            ),
            Node(
                package="tejen_mission",
                executable="pendulum_state_publisher",
                namespace=ns,
                name="pendulum_state_publisher",
                output="screen",
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

        truth_actions.append(
            Node(
                package="tejen_mission",
                executable="m2a_gazebo_joint_truth_bridge",
                namespace=ns,
                name="m2c_joint_truth_bridge",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "gz_topic": f"/{ns}/magnet/joint_detached_truth",
                    "ros_topic": f"/{ns}/magnet/joint_detached_truth",
                    "status_topic": "magnet/joint_truth_bridge_state",
                    "republish_period_s": 0.10,
                }],
            )
        )

        mission_actions.append(
            Node(
                package="tejen_mission",
                executable="online_join_planner",
                namespace=ns,
                name="online_join_planner",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "vehicle_id": vehicle,
                    "mission_mode": "join",
                    "frame_id": "map",
                    "drone_state_topic": f"/{ns}/motion_capture_state",
                    "reference_topic": reference_topic,
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
                    "m2c_ground_reference_enabled": True,
                    "m2c_ground_reference_samples": 61,
                    "committed_trajectory_topic": committed_topic,
                    "committed_trajectory_horizon_s": 10.0,
                    "enable_csv_logging": True,
                    "log_directory": "logs/join_planner",
                    "log_run_label": "m2c_ground",
                }],
            )
        )


    actions.extend([
        TimerAction(period=1.00, actions=platform_actions),
        TimerAction(period=1.20, actions=truth_actions),
        TimerAction(period=1.45, actions=mission_actions),
        TimerAction(period=2.00, actions=[
            Node(
                package="tejen_mission",
                executable="m2c_fleet_manager",
                name="m2c_fleet_manager",
                output="screen",
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
            )
        ]),
    ])
    return LaunchDescription(actions)
