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
        'c1bc_passive_sim.yaml',
    )
    delay = LaunchConfiguration('artificial_planning_delay_ms')
    planner_rate = LaunchConfiguration('planner_rate_hz')
    csv_path = LaunchConfiguration('csv_path')

    return LaunchDescription([
        DeclareLaunchArgument(
            'artificial_planning_delay_ms',
            default_value='0.0',
            description='Artificial wall planning delay used only for the C.1c timing test.',
        ),
        DeclareLaunchArgument(
            'planner_rate_hz',
            default_value='5.0',
            description='Passive planner trigger rate. C.1b/c v3 defaults to 5 Hz from measured ROS runtime.',
        ),
        DeclareLaunchArgument(
            'csv_path',
            default_value='/tmp/r6_3c1bc_passive.csv',
            description='Commissioning diagnostics CSV path.',
        ),
        Node(
            package='tejen_dynamic_planner',
            executable='dynamic_planner_passive_commissioning',
            name='dynamic_planner_passive_commissioning',
            output='screen',
            parameters=[
                config,
                {
                    'artificial_planning_delay_ms': ParameterValue(delay, value_type=float),
                    'planner_rate_hz': ParameterValue(planner_rate, value_type=float),
                    'csv_path': ParameterValue(csv_path, value_type=str),
                },
            ],
        )
    ])
