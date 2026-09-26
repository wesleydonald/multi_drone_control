from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tejen_dynamic_planner'),
        'config',
        'c1d_active_sim.yaml',
    )
    planner_rate = LaunchConfiguration('planner_rate_hz')
    goal_dx = LaunchConfiguration('goal_dx_m')
    csv_path = LaunchConfiguration('csv_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'planner_rate_hz',
            default_value='5.0',
            description='Persistent active planner trigger rate after takeoff/settle.',
        ),
        DeclareLaunchArgument(
            'goal_dx_m',
            default_value='1.0',
            description='First C.1d clear-space goal displacement in +X.',
        ),
        DeclareLaunchArgument(
            'csv_path',
            default_value='/tmp/r6_3c1d_active.csv',
            description='C.1d active bridge diagnostics CSV.',
        ),
        Node(
            package='tejen_dynamic_planner',
            executable='dynamic_planner_active_commissioning',
            name='dynamic_planner_active_commissioning',
            output='screen',
            parameters=[
                config,
                {
                    'planner_rate_hz': ParameterValue(planner_rate, value_type=float),
                    'goal_dx_m': ParameterValue(goal_dx, value_type=float),
                    'csv_path': ParameterValue(csv_path, value_type=str),
                },
            ],
        )
    ])
