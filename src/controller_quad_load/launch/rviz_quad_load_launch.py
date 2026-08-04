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

from controller_quad_load.rviz_config import build_config, drone_mesh_colour

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
    detach = (LaunchConfiguration('detach').perform(context).lower()
              in ('1', 'true', 'yes'))
    attach = (LaunchConfiguration('attach').perform(context).lower()
              in ('1', 'true', 'yes'))
    parent = LaunchConfiguration('parent_model').perform(context)

    # With attach:=true a free approach drone (id n, standalone) also flies. Its pose bridge
    # + mocap come from three_attach_launch (it is NOT nested under lift_system), so the
    # sim-interface loop below stays at `n`. But its VISUALISATION (fleet_viz TF, RobotModel,
    # displays) belongs here -- driven purely off /drone_{n}/motion_capture_state -- so the
    # viz count is n+1.
    n_viz = n + (1 if attach else 0)

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
        name='fleet_viz', parameters=[{'num_drones': n_viz}], output='screen'))

    # Placeholder Telemetry so the ArmPanel's per-drone battery rows populate in
    # sim exactly as they do on hardware (finding F7). On the rig this comes from
    # elrs_interface, so real_io_launch.py must NOT start this node. Drive one
    # drone low to check the warning is visible:
    #   ros2 param set /sim_telemetry voltage_drone_1 13.2
    nodes.append(Node(
        package='simulation_communication', executable='sim_telemetry',
        name='sim_telemetry', parameters=[{'num_drones': n_viz}],
        output='screen'))

    for i in range(n_viz):
        # Point the model's only link at the frame the tracker actually
        # broadcasts, so the mesh follows the live pose.
        urdf = base_urdf.replace('name="base_link"', f'name="drone_{i}_mocap"')
        urdf = urdf.replace('<color rgba="0.15 0.15 0.15 1.0"/>',
                            f'<color rgba="{drone_mesh_colour(i)}"/>')
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
        fh.write(build_config(n_viz, show_actual=show_actual,
                              detach=detach, attach=attach))

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
        # Show the DETACH drone-id selector + button in the ArmPanel (for the
        # dissipative detach controller). Off by default; detach:=true shows the row.
        DeclareLaunchArgument('detach', default_value='false'),
        # Show the ATTACH button in the ArmPanel (arms the approach drone's magnet for the
        # attach flow). Off by default; pass attach:=true with three_attach.sdf.
        DeclareLaunchArgument('attach', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
