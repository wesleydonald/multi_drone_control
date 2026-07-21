"""
rviz_quad_load_launch.py
------------------------
RViz for the cable-suspended payload stack, for ANY fleet size. Run alongside
mpc_quad_load_launch.py with the SAME num_drones:

    ros2 launch controller_quad_load mpc_quad_load_launch.py  num_drones:=4
    ros2 launch controller_quad_load rviz_quad_load_launch.py num_drones:=4

This launch owns the SIM INTERFACE as well as the visuals: the clock/pose
bridges, the mocap emulators and fleet_viz. So you bring this up FIRST, confirm
every drone and the payload are present and their poses are live, and only then
start mpc_quad_load_launch.py to fly them. The control launch deliberately does
not duplicate these -- run this one first or the controllers get no pose.

Shows:
  * the drone AIRFRAME (drone_visualisation's tbs_sourceone_v5 mesh) for each
    drone, coloured per drone and following its live pose
  * the PAYLOAD box (matching the world SDF) and where it has actually been
  * /drone_i/mpc_plan        the MPC's planned horizon (N nodes, 50 Hz)
  * /drone_i/actual_path     where it has actually flown (OFF by default --
                             these are growing trails and clutter the view;
                             show_actual:=true, or tick them on in RViz)
  * /drone_i/trajectory_path its reference (off by default — static clutter)
  * /payload/mpc_plan        the planner's DESIRED load trajectory (horizon)
  * /payload/actual_path     where the payload has actually travelled (OFF by
                             default, same show_actual flag)
  * /payload/marker          the payload box itself
  * the ArmPanel: ARM/DISARM, TAKEOFF and LAND buttons (drone_visualisation)

HOW THE AIRFRAMES TRACK: fleet_viz broadcasts TF map -> drone_i_mocap (and
map -> payload_mocap) straight from the mocap topics. Rather than the static
transform that drone_visualisation's own view_frame_multiple.launch.py uses, we
rename the URDF's single link to drone_i_mocap per drone, so RViz's RobotModel
renders the mesh wherever that live frame is.

MESH SCALE: the STL is the real TBS Source One frame in millimetres (209 x 181 x
35), so mesh_scale 0.001 renders it at its true 209 mm size -- close to the
Gazebo x3's 226 mm motor-to-motor diagonal. The URDF ships with 0.002, which
draws it at 418 mm, about twice life size. Override mesh_scale to taste.

The .rviz config is GENERATED here rather than checked in, because the display
list depends on num_drones. A static config would either miss drones above its
fleet size or carry dead displays below it. Written to /tmp so it never dirties
the repo.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

# per-drone colour: (rviz "r; g; b" for paths, urdf "r g b a" for the mesh)
DRONE_COLOURS = [
    ('31; 119; 180',  '0.12 0.47 0.71 0.9'),   # blue
    ('255; 127; 14',  '1.00 0.50 0.05 0.9'),   # orange
    ('44; 160; 44',   '0.17 0.63 0.17 0.9'),   # green
    ('214; 39; 40',   '0.84 0.15 0.16 0.9'),   # red
    ('148; 103; 189', '0.58 0.40 0.74 0.9'),   # purple
]
PAYLOAD_COLOUR = '255; 215; 0'    # gold


def _path_display(name, topic, colour, width, enabled=True):
    return f"""    - Class: rviz_default_plugins/Path
      Name: {name}
      Enabled: {str(enabled).lower()}
      Topic:
        Value: {topic}
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Color: {colour}
      Line Style: Lines
      Line Width: {width}
      Alpha: 1
      Buffer Length: 1
      Offset: {{X: 0, Y: 0, Z: 0}}
      Pose Style: None"""


def _robot_display(i, enabled=True):
    return f"""    - Class: rviz_default_plugins/RobotModel
      Name: Drone {i} airframe
      Enabled: {str(enabled).lower()}
      Description Source: Topic
      Description Topic:
        Value: /robot_description_{i}
        Depth: 5
        Durability Policy: Transient Local
        Reliability Policy: Reliable
      Description File: ""
      TF Prefix: ""
      Alpha: 1
      Visual Enabled: true
      Collision Enabled: false
      Update Interval: 0"""


def _build_config(n: int, show_actual: bool = False) -> str:
    displays = ["""    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.5
      Plane Cell Count: 20
      Color: 160; 160; 164
      Alpha: 0.5
      Line Style:
        Line Width: 0.03
        Value: Lines
      Plane: XY
      Normal Cell Count: 0
      Offset: {X: 0, Y: 0, Z: 0}
      Reference Frame: <Fixed Frame>""",
                """    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: false
      Show Names: true
      Show Axes: true
      Show Arrows: false
      Marker Scale: 0.3
      Update Interval: 0
      Frame Timeout: 15"""]

    # payload first so it draws under the drones
    displays.append("""    - Class: rviz_default_plugins/Marker
      Name: Payload box
      Enabled: true
      Topic:
        Value: /payload/marker
        Depth: 5
        Durability Policy: Volatile
        Reliability Policy: Reliable
      Namespaces:
        payload: true""")
    displays.append(_path_display(
        'Payload desired', '/payload/mpc_plan', PAYLOAD_COLOUR, 0.03))
    displays.append(_path_display(
        'Payload actual', '/payload/actual_path', '255; 255; 255', 0.015,
        enabled=show_actual))

    for i in range(n):
        c = DRONE_COLOURS[i % len(DRONE_COLOURS)][0]
        displays.append(_robot_display(i))
        displays.append(_path_display(
            f'Drone {i} MPC plan', f'/drone_{i}/mpc_plan', c, 0.02))
        displays.append(_path_display(
            f'Drone {i} actual', f'/drone_{i}/actual_path', c, 0.01,
            enabled=show_actual))
        displays.append(_path_display(
            f'Drone {i} reference', f'/drone_{i}/trajectory_path', c, 0.01,
            enabled=False))

    body = "\n".join(displays)
    return f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded: ~
    Tree Height: 500
  - Class: drone_visualisation/ArmPanel
    Name: ArmPanel
Visualization Manager:
  Class: ""
  Name: root
  Global Options:
    Fixed Frame: map
    Background Color: 48; 48; 48
    Frame Rate: 30
  Displays:
{body}
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Distance: 4
      Focal Point: {{X: 0, Y: 0, Z: 0.5}}
      Pitch: 0.4
      Yaw: 0.8
      Target Frame: <Fixed Frame>
      Near Clip Distance: 0.01
Window Geometry:
  Height: 900
  Width: 1400
  Displays:
    collapsed: false
  ArmPanel:
    collapsed: false
"""


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')

    vis_share = get_package_share_directory('drone_visualisation')
    urdf_path = os.path.join(vis_share, 'urdf', 'frame.urdf')
    with open(urdf_path) as fh:
        base_urdf = fh.read()

    scale = float(LaunchConfiguration('mesh_scale').perform(context))
    show_actual = (LaunchConfiguration('show_actual').perform(context).lower()
                   in ('1', 'true', 'yes'))
    parent = LaunchConfiguration('parent_model').perform(context)

    nodes = [SetParameter(name='use_sim_time', value=True)]

    # ── Sim interface: clock + pose bridges + mocap emulators ──────────────
    # These live here rather than in the control launch so the fleet is visible
    # and publishing BEFORE any controller starts.
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'))
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='payload_pose_bridge',
        arguments=[f'/model/{parent}/model/payload/pose'
                   f'@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']))
    for i in range(n):
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'drone_pose_bridge_{i}',
            arguments=[f'/model/{parent}/model/x3_drone{i}/pose'
                       f'@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']))
        nodes.append(Node(
            package='simulation_communication', executable='payload_mocap_emulator',
            name=f'mocap_{i}',
            parameters=[{'drone_id': i, 'drone_name': f'x3_drone{i}',
                         'parent_model': parent, 'publish_payload': (i == 0)}]))

    # TF for every drone + the payload, plus the payload box and its track
    nodes.append(Node(
        package='simulation_communication', executable='fleet_viz',
        name='fleet_viz', parameters=[{'num_drones': n}], output='screen'))

    for i in range(n):
        # Point the model's only link at the frame the tracker actually
        # broadcasts, so the mesh follows the live pose.
        urdf = base_urdf.replace('name="base_link"', f'name="drone_{i}_mocap"')
        urdf = urdf.replace('<color rgba="0.15 0.15 0.15 1.0"/>',
                            f'<color rgba="{DRONE_COLOURS[i % len(DRONE_COLOURS)][1]}"/>')
        urdf = urdf.replace('scale="0.002 0.002 0.002"',
                            f'scale="{scale} {scale} {scale}"')
        nodes.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name=f'drone_model_{i}',
            parameters=[{'robot_description': urdf, 'use_sim_time': True}],
            # unique topic per drone; the RobotModel displays read these
            remappings=[('robot_description', f'/robot_description_{i}')],
            output='log'))

    cfg_dir = os.path.join(tempfile.gettempdir(), 'quad_load_rviz')
    os.makedirs(cfg_dir, exist_ok=True)
    cfg = os.path.join(cfg_dir, f'quad_load_{n}drone.rviz')
    with open(cfg, 'w') as fh:
        fh.write(_build_config(n, show_actual))

    nodes.append(Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', cfg],
        parameters=[{'use_sim_time': True}],
        output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='2'),
        # The STL is the real TBS frame in mm; 0.001 renders it at its true
        # 209 mm size. The URDF's own 0.002 draws it at ~2x life size.
        DeclareLaunchArgument('mesh_scale', default_value='0.001'),
        DeclareLaunchArgument('parent_model', default_value='lift_system'),
        # Flown/travelled trails. Off by default: they grow for the whole run and
        # clutter the view. The planned paths (MPC plan / payload desired) stay on.
        DeclareLaunchArgument('show_actual', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
