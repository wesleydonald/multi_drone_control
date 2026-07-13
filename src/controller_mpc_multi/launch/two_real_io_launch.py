"""
two_real_io_launch.py  —  TERMINAL 1 (hardware I/O)
---------------------------------------------------
Real-world two-drone I/O layer only: MoCap in + ELRS radio out. Start this FIRST
(in terminal 1), verify the links, then start the controllers with
two_real_control_launch.py in terminal 2.

  * MoCap: ONE motion_capture_publisher_node routes each rigid body to
    /drone_<id>/motion_capture_state (edit RIGID_BODY_TO_DRONE / MOCAP_UDP_* at the
    top of drone_communication/motion_capture_publisher_node.py).
  * ELRS: one elrs_interface per drone, namespaced to /drone_i, each bound to its
    own TX serial device (drone0_serial / drone1_serial; udev symlinks QUAD0/1).

Run:
    ros2 launch controller_mpc_multi two_real_io_launch.py
    #   drone0_serial:=/dev/QUAD0 drone1_serial:=/dev/QUAD1   (defaults)

Verify before starting the controllers:
    ros2 topic hz /drone_0/motion_capture_state          # pose streaming
    ros2 topic echo /drone_0/telemetry --once            # rssi/battery = RF link up
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

N = 2

def generate_launch_description():
    serials = [LaunchConfiguration('drone0_serial'),
               LaunchConfiguration('drone1_serial')]

    nodes = [
        DeclareLaunchArgument('drone0_serial', default_value='/dev/QUAD0',
                              description="drone 0 ELRS TX serial device (udev symlink)"),
        DeclareLaunchArgument('drone1_serial', default_value='/dev/QUAD1',
                              description="drone 1 ELRS TX serial device (udev symlink)"),
        DeclareLaunchArgument('rviz', default_value='true',
                              description="open RViz2 for live pose/trajectory view"),
    ]

    # ── MoCap: ONE publisher node, routes each rigid body to its drone topic ──
    nodes.append(Node(
        package='drone_communication',
        executable='motion_capture_publisher_node',
        name='motion_capture_publisher',
        output='screen',
    ))

    # ── ELRS radio out: one per drone, namespaced, own serial device ─────────
    for i in range(N):
        nodes.append(Node(
            package='drone_communication',
            executable='elrs_interface',
            name='elrs_interface',
            namespace=f'/drone_{i}',
            parameters=[{'serial_port': serials[i]}],
            output='screen',
        ))

    # ── RViz2 (toggle with rviz:=false) ──────────────────────────────────────
    # Loads drone_visualisation's default.rviz if present (falls back to a bare
    # RViz window). It shows the drone frames + trajectory once the controllers
    # (terminal 2) publish them via TrajectoryVisualizer.
    rviz_args = []
    try:
        cfg = os.path.join(get_package_share_directory('drone_visualisation'),
                           'rviz', 'two_real.rviz')
        if os.path.exists(cfg):
            rviz_args = ['-d', cfg]
    except Exception:
        pass
    nodes.append(Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=rviz_args, output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))

    return LaunchDescription(nodes)
