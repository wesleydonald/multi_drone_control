"""Full M1 IRL stack: real pickup object, real aircraft, moving virtual ring."""
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
    visualisation_share = Path(get_package_share_directory('drone_visualisation'))
    config = drone_share / 'config' / 'irl_commissioning.yaml'

    gui = LaunchConfiguration('gui')

    state_pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(drone_share / 'launch' / 'irl_state_pipeline.launch.py')
        )
    )

    virtual_ring = Node(
        package='tejen_mission',
        executable='fake_cooperative_transport_world',
        name='fake_cooperative_transport_world',
        output='screen',
        parameters=[str(config)],
    )

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(dynamic_share / 'launch' / 'c1f_irl_m1_virtual_ring_backend.launch.py')
        ),
        launch_arguments={
            'planner_rate_hz': '5.0',
            'minimum_search_z_m': '0.20',
        }.items(),
    )

    planner = Node(
        package='tejen_mission',
        executable='online_join_planner',
        name='online_join_planner',
        output='screen',
        parameters=[
            str(drone_share / 'config' / 'visual_astar_visual_only.yaml'),
            str(drone_share / 'config' / 'c1f2b_cpp_authority.yaml'),
            str(config),
        ],
    )

    controller = Node(
        package='tejen_mpc',
        executable='main',
        name='tejen_mpc',
        output='screen',
        parameters=[str(config)],
    )

    elrs = Node(
        package='tejen_mission',
        executable='elrs_interface_irl',
        name='elrs_interface_irl',
        output='screen',
        parameters=[str(config)],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(visualisation_share / 'launch' / 'view_frame.launch.py')
        ),
        condition=IfCondition(gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        LogInfo(msg='M1 IRL: REAL ID6 pickup + ID7 drone + ID8 magnet; ID9 ring reserved; delivery ring is VIRTUAL.'),
        LogInfo(msg='Manual ARM/TAKEOFF/DISARM authority is unchanged. This launch never arms automatically.'),
        LogInfo(msg='Mission permits the normal M1 physical magnet-OFF drop at the moving virtual target.'),
        state_pipeline,
        # Subscribe to the one-shot transient-local ring commitment before its
        # publisher starts. DDS durability remains the backup for late joiners.
        TimerAction(period=0.5, actions=[backend]),
        TimerAction(period=1.0, actions=[virtual_ring]),
        TimerAction(period=1.5, actions=[planner]),
        TimerAction(period=2.0, actions=[controller, elrs]),
        TimerAction(period=2.5, actions=[rviz]),
    ])
