"""
mpc_two_real_launch.py
----------------------
REAL-WORLD two-drone MPC formation stack (NO Gazebo). This is the hardware
counterpart of mpc_two_no_cables_launch.py: it keeps the control half unchanged
(2x controller_mpc_multi `controller` + the `main` fleet manager) and swaps every
simulation node for the real I/O from drone_communication:

Prerequisites BEFORE flight (see the code changes that back this launch):
  * MoCap: edit RIGID_BODY_TO_DRONE / MOCAP_UDP_HOST / MOCAP_UDP_PORT at the top
    of drone_communication/motion_capture_publisher_node.py to match your rig.
    Rigid-body IDs are multiples of 10 by convention (10 -> drone 0, 20 -> drone 1).
    The single publisher node routes each body to /drone_<id>/motion_capture_state.
  * ELRS: each drone binds its own TX via the `serial_port` param below
    (drone0_serial / drone1_serial launch args). Two distinct /dev/ttyUSB*.

Run:
    ros2 launch controller_mpc_multi mpc_two_real_launch.py
    # optionally override the radios:
    #   drone0_serial:=/dev/ttyUSB0 drone1_serial:=/dev/ttyUSB1
then drive the fleet:
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: DISARM}"

The controller/fleet nodes pin use_sim_time=False internally, so they run on the
wall clock — correct for real hardware (there is no /clock here).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DRONE_NAMES = ['x3_drone0', 'x3_drone1']
N = len(DRONE_NAMES)

def generate_launch_description():
    drone0_serial = LaunchConfiguration('drone0_serial')
    drone1_serial = LaunchConfiguration('drone1_serial')
    serials = [drone0_serial, drone1_serial]

    nodes = [
        DeclareLaunchArgument('drone0_serial', default_value='/dev/QUAD0',
                              description="drone 0 ELRS TX serial device (udev symlink)"),
        DeclareLaunchArgument('drone1_serial', default_value='/dev/QUAD1',
                              description="drone 1 ELRS TX serial device (udev symlink)"),
    ]

    # ── MoCap: ONE publisher node, routes each rigid body to its drone topic ──
    # (single UDP socket -> can't be namespaced/duplicated; it publishes the
    # absolute /drone_<id>/motion_capture_state topics itself).
    nodes.append(Node(
        package='drone_communication',
        executable='motion_capture_publisher_node',
        name='motion_capture_publisher',
        output='screen',
    ))

    for i in range(N):
        # ── ELRS radio out: /drone_i/ELRSCommand -> this drone's TX ──────────
        # Namespaced so the node's relative 'ELRSCommand'/'telemetry' topics map
        # to /drone_i/... (matching the controller), each on its own serial port.
        nodes.append(Node(
            package='drone_communication',
            executable='elrs_interface',
            name='elrs_interface',
            namespace=f'/drone_{i}',
            parameters=[{'serial_port': serials[i]}],
            output='screen',
        ))

        # ── Per-drone MPC controller (unchanged from sim) ────────────────────
        nodes.append(Node(
            package='controller_mpc_multi',
            executable='controller',
            name=f'controller_{i}',
            parameters=[{'drone_id': i}],
            output='screen',
        ))

    # ── Central fleet manager (unchanged from sim) ───────────────────────────
    nodes.append(Node(
        package='controller_mpc_multi',
        executable='main',
        name='central_controller',
        parameters=[{'num_drones': N}],
        output='screen',
    ))

    return LaunchDescription(nodes)
