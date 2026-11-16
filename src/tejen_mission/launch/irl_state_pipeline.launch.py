from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory('tejen_mission')) / 'config' / 'irl_commissioning.yaml'
    return LaunchDescription([
        LogInfo(msg='IRL state pipeline uses config/irl_commissioning.yaml. Edit that one file before lab use.'),
        LogInfo(msg='IRL attachment Bool is inferred from magnet ON + LIVE ID6 pickup proximity/relative-speed/dwell; it is NOT sensed.'),
        Node(
            package='tejen_mission',
            executable='motion_capture_publisher_irl',
            name='motion_capture_publisher_irl',
            output='screen',
            parameters=[str(config)],
        ),
        Node(
            package='tejen_mission',
            executable='magnet_attachment_manager_irl',
            name='magnet_attachment_manager_irl',
            output='screen',
            parameters=[str(config)],
        ),
    ])
