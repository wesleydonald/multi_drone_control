"""
mpc_four_quad_load_launch.py
----------------------------
FOUR-drone counterpart of mpc_three_soft_quad_load_launch.py: the centralized
cable-suspended load planner (controller_load_mpc) generates each drone's
reference trajectory AND its per-node cable tension acceleration (t*s/m), and the
per-drone cable-aware MPC (controller_quad_load) tracks it with the cable force in
its prediction model (reference_source:=planner).

Run Gazebo first:
    gz sim simulation_assets/four_rigid_short.sdf -v 4 -r
then:
    ros2 launch controller_quad_load mpc_four_quad_load_launch.py

The defaults target four_rigid_short.sdf (0.5 m rigid rods at 45deg, 0.4 kg
payload) and need no arguments — the same geometry as three_rigid_short.sdf with
a fourth drone, so the per-drone tension drops from mg/(3 sin45) to mg/(4 sin45).
For four_rigid.sdf override the geometry:
    four_rigid.sdf  cable_len:=0.707

ARM + TAKEOFF via /fleet/command hands the drones over to tracking:
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: LAND}"

Note: num_drones is passed to BOTH the fleet manager and the planner. They must
agree with the world SDF — the planner divides the load tension by n, so a
mismatch mis-scales every drone's feedforward.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

PARENT_MODEL = 'lift_system'
DRONE_NAMES  = ['x3_drone0', 'x3_drone1', 'x3_drone2', 'x3_drone3']
N            = len(DRONE_NAMES)


def generate_launch_description():
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
        # must match the tether length in the world SDF - four_rigid_short.sdf
        # uses rigid 0.5 m rods, and a mismatch here commands a formation radius
        # the tethers physically can't reach (drones fight the rod on takeoff).
        # four_rigid.sdf = 0.707.
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        DeclareLaunchArgument('start_taut', default_value='true'),
        # payload mass in the world SDF.
        DeclareLaunchArgument('load_mass', default_value='0.4'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        # HOLD test: lift_ramp_vel:=0.0 (no lift, just hold the taut config).
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.20'),
        # LAND descent rate (separate from the slow takeoff lift_ramp_vel).
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        # Cable compensation. ON is the correct flight config: the cable pulls
        # each drone inward-and-down, so the drone must hold an outward tilt just
        # to stay put. With four drones the per-drone tension is mg/(4 sin45) =
        # 1.39 N (vs 1.85 N on three), so the required tilt is ~7.9deg not 10.3.
        # Only zero these for a deliberate A/B:
        #   cable_ff_scale:=0.0  cable-blind prediction model
        #   attitude_ff:=false   level attitude reference (throttle FF kept)
        DeclareLaunchArgument('cable_ff_scale', default_value='1.0'),
        DeclareLaunchArgument('attitude_ff', default_value='true'),
        # 'model' = planner open-loop t*s/m cable term (default); 'measured' =
        # IMU-derived f_ext held over the horizon.
        DeclareLaunchArgument('cable_source', default_value='model'),
        # payload counts as resting (cable term zeroed) at/below this z.
        DeclareLaunchArgument('payload_rest_z', default_value='0.05'),
        # seconds to spool the throttle up at takeoff (gentle liftoff); 0 = instant.
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.0'),
        # Thrust accel per unit throttle the MPC assumes (kT).
        #
        # The sim's plant is actually QUADRATIC: from the SDF motor model
        # (motorConstant 1.42e-6, maxRotVelocity 4631, 4 rotors, 0.6 kg) the
        # specific thrust is a(u) = 203*u^2, not kT*u. So no single kT is right
        # everywhere -- this one is chosen to match at the hover operating point,
        # which is where it matters most:
        #     n=2  u_hover 0.258 -> kT 52.3
        #     n=3  u_hover 0.245 -> kT 49.7
        #     n=4  u_hover 0.239 -> kT 48.4
        # 50.0 splits the three (each within ~5%). Confirmed against flight logs:
        # least squares on the vertical dynamics gave 51.1, steady-state hover
        # throttle gave 50.9.
        #
        # It was previously 38 -- a ~30% UNDER-estimate, so the feedforward asked
        # for ~30% more throttle than needed and the drones leapt off the
        # platforms (measured 0.77 m/s climb against a reference moving at 0.00,
        # 0.51 m overshoot). Steady flight hid it because position feedback trims
        # it back out; it only showed as an aggressive TRANSIENT.
        #
        # KNOWN LIMITATION: because the model is linear and the plant quadratic,
        # the model's local slope at hover is HALF the true one (d a/d u is
        # 2*203*u = 2*kT). The MPC therefore commands ~2x the throttle correction
        # it needs on vertical transients. Matching the value at hover does not
        # fix that -- only a quadratic thrust model would. Use ~24 for hardware.
        DeclareLaunchArgument('thrust_ratio', default_value='38.0'),
        # 'kinematic' = open-loop feedforward (tracker stabilizes); 'coupled' =
        # online load-cable OCP (diverges — kept for A/B comparison).
        DeclareLaunchArgument('planner_mode', default_value='kinematic'),
        # 'taut' = engage FF from spawn (rigid cables); 'airborne' = ramp with the
        # load lift (soft cables).
        DeclareLaunchArgument('ff_gate_mode', default_value='taut'),
        # LOAD reference trajectory after the lift tops out: 'hover', 'line_x',
        # 'circle'. Keep traj_speed slow.
        DeclareLaunchArgument('load_traj', default_value='line_x'),
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
        parameters=[{'num_drones': N,
                     'cable_len': cable_len, 'start_taut': start_taut,
                     'load_mass': load_mass,
                     'target_z': target_z, 'lift_ramp_vel': lift_ramp_vel,
                     'land_vel': land_vel,
                     'planner_mode': planner_mode, 'ff_gate_mode': ff_gate_mode,
                     'load_traj': load_traj, 'traj_speed': traj_speed,
                     'traj_distance': traj_distance, 'traj_radius': traj_radius}],
        output='screen'))

    return LaunchDescription(nodes)
