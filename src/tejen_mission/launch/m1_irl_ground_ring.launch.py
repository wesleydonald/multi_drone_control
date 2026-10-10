"""Opt-in M1 IRL stack with one measured stationary ring (ID8)."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    drone_share = Path(get_package_share_directory('tejen_mission'))
    dynamic_share = Path(get_package_share_directory('tejen_dynamic_planner'))
    visual_share = Path(get_package_share_directory('drone_visualisation'))
    base = drone_share / 'config' / 'irl_commissioning.yaml'
    ground_ring = drone_share / 'config' / 'm1_ground_ring.yaml'

    mocap = Node(
        package='tejen_mission', executable='motion_capture_publisher_irl',
        name='motion_capture_publisher_irl', output='screen',
        parameters=[str(base), str(ground_ring)],
    )
    pickup_attachment = Node(
        package='tejen_mission', executable='magnet_attachment_manager_irl',
        name='magnet_attachment_manager_irl', output='screen', parameters=[str(base)],
    )
    ring_reference = Node(
        package='tejen_mission', executable='fake_cooperative_transport_world',
        name='fake_cooperative_transport_world', output='screen',
        parameters=[str(base), str(ground_ring)],
    )
    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(dynamic_share / 'launch' / 'c1f_irl_m1_virtual_ring_backend.launch.py')
        ),
        launch_arguments={
            'planner_rate_hz': '5.0',
            'minimum_search_z_m': '0.20',
            'ring_commitment_topic': '/irl/ring/committed_trajectory',
        }.items(),
    )
    planner = Node(
        package='tejen_mission', executable='online_join_planner',
        name='online_join_planner', output='screen',
        parameters=[
            str(drone_share / 'config' / 'visual_astar_visual_only.yaml'),
            str(drone_share / 'config' / 'c1f2b_cpp_authority.yaml'),
            str(base),
            str(drone_share / 'config' / 'm1_measured_tip_pickup.yaml'),
            str(ground_ring),
            str(drone_share / 'config' / 'm1_measured_tip_tracking.yaml'),
            {'pickup_alignment_mode': 'measured_tip_tiered',
             'target_alignment_mode': LaunchConfiguration('target_alignment_mode')},
        ],
    )
    controller = Node(
        package='tejen_mpc', executable='main',
        name='tejen_mpc', output='screen', parameters=[str(base)],
    )
    elrs = Node(
        package='tejen_mission', executable='elrs_interface_irl',
        name='elrs_interface_irl', output='screen', parameters=[str(base)],
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(visual_share / 'launch' / 'view_frame.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('target_alignment_mode', default_value='legacy',
                              choices=['legacy', 'measured_tip_trim']),
        LogInfo(msg=['M1 local target alignment: ', LaunchConfiguration('target_alignment_mode')]),
        LogInfo(msg='M1 GROUND RING: real pickup ID6, quad ID12, magnet ID22, measured ring ID8.'),
        LogInfo(msg='Ring contact is an UNVERIFIED trial. ARM/TAKEOFF and LAND remain manual.'),
        mocap,
        TimerAction(period=0.5, actions=[backend]),
        TimerAction(period=1.0, actions=[ring_reference]),
        TimerAction(period=1.5, actions=[planner, pickup_attachment]),
        TimerAction(period=2.0, actions=[controller, elrs]),
        TimerAction(period=2.5, actions=[rviz]),
    ])
