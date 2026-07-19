"""
mpc_three_soft_quad_load_launch.py
----------------------------------
PLANNER-DRIVEN, CABLE-AWARE stack for three_soft.sdf: the centralized
cable-suspended load planner (controller_load_mpc) generates each drone's
reference trajectory AND its per-node cable tension acceleration (t*s/m), and the
per-drone cable-aware MPC (controller_quad_load) tracks it with the cable force in
its prediction model (reference_source:=planner).

vs controller_mpc_multi/mpc_three_soft_planner_launch.py (cable-blind tracker),
this swaps the controllers + fleet manager to the controller_quad_load package.

Run Gazebo first:
    gz sim simulation_assets/three_soft.sdf -v 4 -r
then:
    ros2 launch controller_quad_load mpc_three_soft_quad_load_launch.py
The planner streams references continuously; ARM + TAKEOFF via /fleet/command
hands the drones over to tracking it:
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"

Notes:
  * planner codegen is isolated (c_generated_code_load_planner/), and the
    cable-aware tracker codegen is isolated (c_generated_code_quad_load/), so
    nothing clashes with controller_mpc_multi's quad_dynamics solver.
  * until the planner streams a reference, controllers hold armed-idle.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

PARENT_MODEL = 'lift_system'
DRONE_NAMES  = ['x3_drone0', 'x3_drone1', 'x3_drone2']
N            = len(DRONE_NAMES)


def generate_launch_description():
    # Geometry defaults target three_soft_paper.sdf (elevated, TAUT 1.0 m cables
    # at ~45deg). For the ground-start slack three_soft.sdf, launch with
    #   cable_len:=0.6 start_taut:=false
    cable_len     = ParameterValue(LaunchConfiguration('cable_len'), value_type=float)
    start_taut    = ParameterValue(LaunchConfiguration('start_taut'), value_type=bool)
    load_mass     = ParameterValue(LaunchConfiguration('load_mass'), value_type=float)
    target_z      = ParameterValue(LaunchConfiguration('target_z'), value_type=float)
    lift_ramp_vel = ParameterValue(LaunchConfiguration('lift_ramp_vel'), value_type=float)
    land_vel      = ParameterValue(LaunchConfiguration('land_vel'), value_type=float)
    planner_mode  = LaunchConfiguration('planner_mode')
    ff_gate_mode  = LaunchConfiguration('ff_gate_mode')
    load_traj     = LaunchConfiguration('load_traj')
    traj_speed    = ParameterValue(LaunchConfiguration('traj_speed'), value_type=float)
    traj_distance = ParameterValue(LaunchConfiguration('traj_distance'), value_type=float)
    traj_radius   = ParameterValue(LaunchConfiguration('traj_radius'), value_type=float)
    cable_ff_scale = ParameterValue(LaunchConfiguration('cable_ff_scale'), value_type=float)
    attitude_ff    = ParameterValue(LaunchConfiguration('attitude_ff'), value_type=bool)
    cable_source   = LaunchConfiguration('cable_source')
    payload_rest_z = ParameterValue(LaunchConfiguration('payload_rest_z'), value_type=float)
    takeoff_spool_s = ParameterValue(LaunchConfiguration('takeoff_spool_s'), value_type=float)
    thrust_ratio    = ParameterValue(LaunchConfiguration('thrust_ratio'), value_type=float)

    nodes = [
        # must match the tether length in the world SDF - three_rigid_short.sdf
        # uses rigid 0.5 m rods, and a mismatch here commands a formation radius
        # the tethers physically can't reach (drones fight the rod on takeoff).
        DeclareLaunchArgument('cable_len', default_value='1.0'),
        DeclareLaunchArgument('start_taut', default_value='true'),
        # payload mass in the world SDF: 0.4 for three_soft/three_rigid_short,
        # 0.1 for three_soft_paper.
        DeclareLaunchArgument('load_mass', default_value='0.4'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        # HOLD test: lift_ramp_vel:=0.0 (no lift, just hold the taut config).
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.20'),
        # LAND descent rate (separate from the slow takeoff lift_ramp_vel).
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        # Tracker diagnostic knobs: cable_ff_scale:=0.0 = cable-blind model;
        # attitude_ff:=false = level attitude reference (keep throttle FF).
        DeclareLaunchArgument('cable_ff_scale', default_value='0.0'),
        DeclareLaunchArgument('attitude_ff', default_value='false'),
        # 'model' = planner open-loop t*s/m cable term (default); 'measured' =
        # IMU-derived f_ext held over the horizon (Part 2c A/B).
        DeclareLaunchArgument('cable_source', default_value='model'),
        # payload counts as resting (cable term zeroed) at/below this z; set a
        # bit above the payload's on-ground height for the world in use.
        DeclareLaunchArgument('payload_rest_z', default_value='0.05'),
        # seconds to spool the throttle up at takeoff (gentle liftoff); 0 = instant.
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.0'),
        # thrust accel per unit throttle the MPC assumes. 24 ~= hardware; raise
        # toward the sim's real value (~40) so the drones don't over-throttle.
        DeclareLaunchArgument('thrust_ratio', default_value='38.0'),
        # 'kinematic' = open-loop feedforward (tracker stabilizes); 'coupled' =
        # online load-cable OCP (diverges — kept for A/B comparison).
        DeclareLaunchArgument('planner_mode', default_value='kinematic'),
        # 'taut' = engage FF from spawn (rigid cables); 'airborne' = ramp with the
        # load lift (soft cables).
        DeclareLaunchArgument('ff_gate_mode', default_value='taut'),
        # LOAD reference trajectory after the lift tops out: 'hover' (current
        # behaviour), 'line_x' (+x translate), 'circle'. Keep traj_speed slow.
        DeclareLaunchArgument('load_traj', default_value='circle'),
        DeclareLaunchArgument('traj_speed', default_value='0.4'),
        DeclareLaunchArgument('traj_distance', default_value='1.0'),
        DeclareLaunchArgument('traj_radius', default_value='0.5'),
        SetParameter(name='use_sim_time', value=True),
    ]

    # ── Clock + payload pose bridges ───────────────────────────────────────
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'))
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='payload_pose_bridge',
        arguments=[f'/model/{PARENT_MODEL}/model/payload/pose'
                   f'@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']))

    for i, drone_name in enumerate(DRONE_NAMES):
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'drone_pose_bridge_{i}',
            arguments=[f'/model/{PARENT_MODEL}/model/{drone_name}/pose'
                       f'@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']))
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'motor_bridge_{i}',
            arguments=[f'/{drone_name}/gazebo/command/motor_speed'
                       f'@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']))
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'imu_bridge_{i}',
            arguments=[f'/{drone_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            remappings=[(f'/{drone_name}/imu', f'/drone_{i}/imu')]))
        nodes.append(Node(
            package='simulation_communication', executable='payload_betaflight_comm',
            name=f'bf_comm_{i}',
            parameters=[{'drone_id': i, 'drone_name': drone_name,
                         'parent_model': PARENT_MODEL}]))
        nodes.append(Node(
            package='simulation_communication', executable='payload_mocap_emulator',
            name=f'mocap_{i}',
            parameters=[{'drone_id': i, 'drone_name': drone_name,
                         'parent_model': PARENT_MODEL, 'publish_payload': (i == 0)}]))
        # per-drone CABLE-AWARE MPC tracker — track the planner reference
        nodes.append(Node(
            package='controller_quad_load', executable='controller',
            name=f'controller_{i}',
            parameters=[{'drone_id': i, 'reference_source': 'planner',
                         'cable_ff_scale': cable_ff_scale,
                         'attitude_ff': attitude_ff,
                         'cable_source': cable_source,
                         'payload_rest_z': payload_rest_z,
                         'takeoff_spool_s': takeoff_spool_s,
                         'thrust_ratio': thrust_ratio}],
            output='screen'))

    # ── Central fleet manager ──────────────────────────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': N}], output='screen'))

    # ── Centralized cable-suspended load planner ───────────────────────────
    nodes.append(Node(
        package='controller_load_mpc', executable='planner', name='load_planner',
        parameters=[{'cable_len': cable_len, 'start_taut': start_taut,
                      'load_mass': load_mass,
                      'target_z': target_z, 'lift_ramp_vel': lift_ramp_vel,
                      'land_vel': land_vel,
                      'planner_mode': planner_mode, 'ff_gate_mode': ff_gate_mode,
                      'load_traj': load_traj, 'traj_speed': traj_speed,
                      'traj_distance': traj_distance, 'traj_radius': traj_radius}],
        output='screen'))

    return LaunchDescription(nodes)
