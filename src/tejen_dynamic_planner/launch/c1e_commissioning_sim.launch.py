from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    dynamic_share = Path(get_package_share_directory('tejen_dynamic_planner'))

    # C1E intentionally launches only the simulation interfaces it consumes.
    # The legacy tejen_betaflight_linear_simulation_launch.py also starts camera bridges,
    # a dynamic-pose bridge, and the obsolete pendulum_state_listener. Those remain
    # untouched for older workflows but are not part of C1E commissioning.
    x3_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='c1e_x3_pose_bridge',
        arguments=[
            '/model/x3/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V'
        ],
    )

    motor_command_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='c1e_motor_command_bridge',
        arguments=[
            '/X3/gazebo/command/motor_speed'
            '@actuator_msgs/msg/Actuators]ignition.msgs.Actuators'
        ],
    )

    payload_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='c1e_payload_pose_bridge',
        arguments=[
            '/model/payload_model/pose'
            '@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
        ],
    )

    motion_capture = Node(
        package='simulation_communication',
        executable='tejen_motion_capture_emulator',
        name='motion_capture_emulator',
        output='screen',
        parameters=[{
            'target_object_id': 7,
            'enable_orientation_bias': False,
        }],
    )

    betaflight = Node(
        package='simulation_communication',
        executable='tejen_betaflight_communication',
        name='betaflight_communication',
        output='screen',
        parameters=[{
            'rates_d_val': 100.0,
            'rates_f_val': 100.0,
            'rates_g_val': 0.0,
        }],
    )

    pendulum = Node(
        package='tejen_mission',
        executable='pendulum_state_publisher',
        name='pendulum_state_publisher',
        output='screen',
        parameters=[{
            'x3_pose_topic': '/model/x3/pose',
            'magnet_index': 0,
            'attachment_index': 5,
            'drone_index': 7,
            'x3_link_poses_are_relative': True,
        }],
    )

    magnet_manager = Node(
        package='tejen_mission',
        executable='magnet_attachment_manager',
        name='magnet_attachment_manager',
        output='screen',
        parameters=[{
            'enable_elrs_magnet_output': False,
            'attachment_mode': 'fixed_joint',
            'command_backend': 'gz_cli',
            'magnet_tip_pose_topic': '/magnet_tip_pose',
            'magnet_tip_pose_msg_type': 'pose_stamped',
            'object_pose_topic': '/model/payload_model/pose',
            'object_pose_index': 1,
            'use_fallback_object_pose': False,
            'attach_radius': 0.18,
            'attach_speed_threshold': 1.0,
            'attach_dwell_time_s': 0.0,
        }],
    )

    planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(dynamic_share / 'launch' / 'c1e_static_sim.launch.py')
        )
    )

    return LaunchDescription([
        LogInfo(msg='C1E minimal staged stack: required Gazebo bridges only.'),
        LogInfo(
            msg='Do not run separate Betaflight, mocap, pendulum, magnet-manager, '
                'or C1E planner processes at the same time.'
        ),
        x3_pose_bridge,
        motor_command_bridge,
        payload_pose_bridge,
        TimerAction(
            period=0.25,
            actions=[
                LogInfo(msg='C1E bridges started: starting mocap + Betaflight communication.'),
                motion_capture,
                betaflight,
            ],
        ),
        TimerAction(
            period=0.75,
            actions=[
                LogInfo(msg='C1E state support: starting pendulum + magnet manager.'),
                pendulum,
                magnet_manager,
            ],
        ),
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='C1E state path settled: starting planner/reference owner.'),
                planner,
            ],
        ),
    ])
