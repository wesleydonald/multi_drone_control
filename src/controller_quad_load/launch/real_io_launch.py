"""
real_io_launch.py  —  TERMINAL 1 (hardware I/O + visualisation)
---------------------------------------------------------------
Real-world cable-suspended-payload I/O layer only: MoCap in + ELRS radio out +
RViz, for ANY fleet size (num_drones). Start this FIRST (terminal 1), verify the
links, then start the controllers + planner with real_control_launch.py in
terminal 2. This is the hardware twin of mpc_quad_load_launch.py's sim I/O +
rviz_quad_load_launch.py's visuals -- no Gazebo bridges, no betaflight_comm, no
mocap emulators; the real MoCap and radios replace them, and the SAME fleet_viz
draws the scene straight from the mocap topics.

  * MoCap: ONE motion_capture_publisher_node routes each rigid body to its topic:
      - each drone body  -> /drone_<id>/motion_capture_state  (the trackers read)
      - the payload body -> /payload/motion_capture_state      (planner + trackers)
    The routing table lives at the top of
    drone_communication/motion_capture_publisher_node.py:
      RIGID_BODY_TO_DRONE = {10: 0, 11: 1, ...}   # add a row per drone
      PAYLOAD_RIGID_BODY_ID = 8                    # the payload's rigid body
      MOCAP_UDP_HOST / MOCAP_UDP_PORT              # your mocap stream endpoint
    num_drones controls how many ELRS links, drone models and RViz displays
    spin up; the
    rigid-body map must be edited to match (the node routes bodies by ID).
  * ELRS: one elrs_interface per drone, namespaced to /drone_i, each bound to its
    own TX serial device (default /dev/QUAD<i>; udev symlinks QUAD0/1/...).
  * fleet_viz: from the mocap topics alone it broadcasts TF map -> drone_i_mocap
    and map -> payload_mocap, and publishes the PAYLOAD box (/payload/marker). No
    Gazebo needed -- it is pure mocap in, TF + marker out.
  * RViz: opens automatically (rviz:=false to suppress) and shows, for exactly
    num_drones drones:
      - one coloured drone airframe mesh per drone, following its live pose
      - the payload box + its desired/actual track
      - each drone's MPC plan (once terminal 2 is up)
      - the fleet ArmPanel: ARM/DISARM, TAKEOFF, LAND buttons, plus one
        armed-state + battery row per drone. DETACH/ATTACH are hidden here --
        they drive the dissipative/magnet stacks, which this launch does not
        run (see rviz_quad_load_launch.py detach:=/attach:= for those).
    The config is generated at launch time so it scales to any num_drones.

Run:
    ros2 launch controller_quad_load real_io_launch.py num_drones:=2
    #   drone0_serial:=/dev/QUAD0 drone1_serial:=/dev/QUAD1 ...   (defaults)

Verify before starting the controllers (terminal 2):
    ros2 topic hz /drone_0/motion_capture_state          # drone pose streaming
    ros2 topic hz /payload/motion_capture_state          # payload pose streaming
    ros2 topic echo /drone_0/telemetry --once            # rssi/battery = RF link up
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from controller_quad_load.rviz_config import build_config, drone_mesh_colour

def _args():
    return [
        # Fleet size. Determines how many ELRS radio links, drone models and RViz
        # displays start. The MoCap rigid-body -> drone map (RIGID_BODY_TO_DRONE in
        # the publisher node) must be edited to match -- the node routes bodies by
        # ID, not by this count.
        DeclareLaunchArgument('num_drones', default_value='2'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='open RViz2 (drone models + payload + ArmPanel)'),
        # The STL is the real TBS frame in mm; 0.001 renders it at its true 209 mm
        # size. The URDF's own 0.002 draws it at ~2x life size.
        DeclareLaunchArgument('mesh_scale', default_value='0.001'),
        # Flown/travelled trails. Off by default: they grow for the whole run and
        # clutter the view. The planned paths (MPC plan / payload desired) stay on.
        DeclareLaunchArgument('show_actual', default_value='false'),
    ]


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')

    scale = float(LaunchConfiguration('mesh_scale').perform(context))
    show_actual = (LaunchConfiguration('show_actual').perform(context).lower()
                   in ('1', 'true', 'yes'))

    vis_share = get_package_share_directory('drone_visualisation')
    with open(os.path.join(vis_share, 'urdf', 'frame.urdf')) as fh:
        base_urdf = fh.read()

    nodes = []

    # ── MoCap: ONE publisher node. Routes each drone body to
    #    /drone_<id>/motion_capture_state and the payload body
    #    (PAYLOAD_RIGID_BODY_ID) to /payload/motion_capture_state -- the topics the
    #    trackers and the load planner read directly.
    nodes.append(Node(
        package='drone_communication',
        executable='motion_capture_publisher_node',
        name='motion_capture_publisher',
        output='screen',
    ))

    # ── fleet_viz: TF map -> drone_i_mocap / payload_mocap + payload box, all
    #    from the mocap topics (no Gazebo). The RobotModels below attach to the
    #    drone_i_mocap frames it broadcasts.
    nodes.append(Node(
        package='simulation_communication', executable='fleet_viz',
        name='fleet_viz', parameters=[{'num_drones': n}], output='screen'))

    # Per-drone ELRS TX serial args (default QUAD<i>) + a robot_state_publisher
    # serving that drone's coloured mesh, its root link renamed to the fleet_viz
    # frame drone_<i>_mocap so the airframe follows the live pose. Declared here
    # (not in _args) because the count depends on the resolved num_drones.
    for i in range(n):
        arg = f'drone{i}_serial'
        nodes.append(DeclareLaunchArgument(
            arg, default_value=f'/dev/QUAD{i}',
            description=f'drone {i} ELRS TX serial device (udev symlink)'))

        # ── ELRS radio out: namespaced, own serial device ────────────────────
        nodes.append(Node(
            package='drone_communication',
            executable='elrs_interface',
            name='elrs_interface',
            namespace=f'/drone_{i}',
            parameters=[{'serial_port': LaunchConfiguration(arg)}],
            output='screen',
        ))

        # ── RViz drone model on /robot_description_<i> ────────────────────────
        urdf = base_urdf.replace('name="base_link"', f'name="drone_{i}_mocap"')
        urdf = urdf.replace('<color rgba="0.15 0.15 0.15 1.0"/>',
                            f'<color rgba="{drone_mesh_colour(i)}"/>')
        urdf = urdf.replace('scale="0.002 0.002 0.002"',
                            f'scale="{scale} {scale} {scale}"')
        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name=f'drone_model_{i}',
            parameters=[{'robot_description': urdf}],
            remappings=[('robot_description', f'/robot_description_{i}')],
            output='log'))

    # ── RViz2: config generated for exactly n drones (rviz:=false to suppress) ─
    cfg_dir = os.path.join(tempfile.gettempdir(), 'quad_load_rviz')
    os.makedirs(cfg_dir, exist_ok=True)
    cfg = os.path.join(cfg_dir, f'real_io_{n}drone.rviz')
    with open(cfg, 'w') as fh:
        fh.write(build_config(n, show_actual=show_actual,
                              detach=False, attach=False))

    nodes.append(Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', cfg], output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])
