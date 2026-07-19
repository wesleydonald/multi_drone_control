"""
mpc_two_quad_load_launch.py
---------------------------
TWO-drone counterpart of mpc_four_quad_load_launch.py: the centralized
cable-suspended load planner (controller_load_mpc) generates each drone's
reference trajectory AND its per-node cable tension acceleration (t*s/m), and the
per-drone cable-aware MPC (controller_quad_load) tracks it with the cable force in
its prediction model (reference_source:=planner).

Run Gazebo first:
    gz sim simulation_assets/two_rigid_short.sdf -v 4 -r
then:
    ros2 launch controller_quad_load mpc_two_quad_load_launch.py

The defaults target two_rigid_short.sdf (0.5 m rigid rods at 45deg, 0.4 kg
payload) — the same geometry as three_/four_rigid_short.sdf with the drones
opposed along +/-x. For two_rigid.sdf override:
    two_rigid.sdf  cable_len:=0.707

TWO DRONES IS THE HARDEST CASE, for two reasons:

  1. Per-drone load is highest. Tension is mg/(n sin45), so it rises as n falls:
       n=2  2.77 N  -> 14.0 deg standing tilt, throttle ~0.36
       n=3  1.85 N  -> 10.3 deg,               throttle ~0.32
       n=4  1.39 N  ->  8.1 deg,               throttle ~0.30
     Still well inside authority, but the tilt feedforward matters MORE here, so
     do not run this world with cable_ff_scale:=0 / attitude_ff:=false except as
     a deliberate A/B — it will sag inboard far worse than the 3- or 4-drone case.

  2. The payload has an UNCONTROLLED rotational DOF. Two cables define a line;
     the payload is free to rotate about it and no combination of drone positions
     can resist that. It is stable only because the CoG hangs attach_z (25 mm)
     below the attach line, giving a weak pendulum at ~1.1 Hz that the ball-joint
     damping alone has to bleed off. Expect visible payload roll about x,
     especially after a lateral direction change. The planner does not model or
     damp it. If it is a problem, raise attach_z in the world (moves the pivot
     further above the CoG, stiffening the restoring torque).

ARM + TAKEOFF via /fleet/command:
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
DRONE_NAMES  = ['x3_drone0', 'x3_drone1']
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
        # must match the tether length in the world SDF - two_rigid_short.sdf uses
        # rigid 0.5 m rods, and a mismatch here commands a formation radius the
        # tethers physically can't reach (drones fight the rod on takeoff).
        # two_rigid.sdf = 0.707.
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        DeclareLaunchArgument('start_taut', default_value='true'),
        # payload mass in the world SDF.
        DeclareLaunchArgument('load_mass', default_value='0.4'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        # HOLD test: lift_ramp_vel:=0.0 (no lift, just hold the taut config).
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.20'),
        # LAND descent rate (separate from the slow takeoff lift_ramp_vel).
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        # Cable compensation. See the header: at n=2 the standing tilt is 14 deg,
        # the largest of any fleet size, so these matter more here than anywhere.
        # Only zero them for a deliberate A/B:
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
        DeclareLaunchArgument('thrust_ratio', default_value='44.0'),
        # 'kinematic' = open-loop feedforward (tracker stabilizes); 'coupled' =
        # online load-cable OCP (diverges — kept for A/B comparison).
        DeclareLaunchArgument('planner_mode', default_value='kinematic'),
        # 'taut' = engage FF from spawn (rigid cables); 'airborne' = ramp with the
        # load lift (soft cables).
        DeclareLaunchArgument('ff_gate_mode', default_value='taut'),
        # LOAD reference trajectory after the lift tops out: 'hover', 'line_x'
        # (continuous shuttle), 'circle'. Start with hover on this world — see the
        # header note on the free payload roll DOF.
        DeclareLaunchArgument('load_traj', default_value='line_x'),
        DeclareLaunchArgument('traj_speed', default_value='0.2'),
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
