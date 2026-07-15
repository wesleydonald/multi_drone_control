"""
mpc_two_quad_load_launch.py
---------------------------
TWO-DRONE cable-aware stack (Stage 4 — 2-drone carry risk gate). Same planner +
cable-aware tracker as mpc_three_soft_quad_load_launch.py, but for a 2-drone
fleet. Use with an elevated 2-drone world matching the planner geometry, e.g.
simulation_assets/two_rigid_paper.sdf (generate with
`python3 simulation_assets/generate_rigid_world.py --n 2 --out two_rigid_paper.sdf`).

The two attach points are at 0/180deg (opposite), load centred between the drones —
the balanced phase-1 geometry of the 2-then-3 reconfiguration. Note two cables
leave the load's rotational / out-of-plane DOFs unactuated; this launch is the
risk-gate for whether that stays bounded.

Run:
    gz sim simulation_assets/two_rigid_paper.sdf -v1 -r
    ros2 launch controller_quad_load mpc_two_quad_load_launch.py \
        planner_mode:=geometric cable_len:=1.0 start_taut:=true
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub --once /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

PARENT_MODEL = 'lift_system'
DRONE_NAMES  = ['x3_drone0', 'x3_drone1']   # 2-drone fleet
N            = len(DRONE_NAMES)


def generate_launch_description():
    cable_len     = ParameterValue(LaunchConfiguration('cable_len'), value_type=float)
    start_taut    = ParameterValue(LaunchConfiguration('start_taut'), value_type=bool)
    target_z      = ParameterValue(LaunchConfiguration('target_z'), value_type=float)
    lift_ramp_vel = ParameterValue(LaunchConfiguration('lift_ramp_vel'), value_type=float)
    planner_mode  = LaunchConfiguration('planner_mode')
    ff_gate_mode  = LaunchConfiguration('ff_gate_mode')
    load_traj     = LaunchConfiguration('load_traj')
    traj_speed    = ParameterValue(LaunchConfiguration('traj_speed'), value_type=float)
    traj_distance = ParameterValue(LaunchConfiguration('traj_distance'), value_type=float)
    traj_radius   = ParameterValue(LaunchConfiguration('traj_radius'), value_type=float)
    cable_ff_scale = ParameterValue(LaunchConfiguration('cable_ff_scale'), value_type=float)
    attitude_ff    = ParameterValue(LaunchConfiguration('attitude_ff'), value_type=bool)
    cable_source   = LaunchConfiguration('cable_source')
    thrust_ratio   = ParameterValue(LaunchConfiguration('thrust_ratio'), value_type=float)
    kp_load_xy = ParameterValue(LaunchConfiguration('kp_load_xy'), value_type=float)
    kd_load_xy = ParameterValue(LaunchConfiguration('kd_load_xy'), value_type=float)
    ki_load_xy = ParameterValue(LaunchConfiguration('ki_load_xy'), value_type=float)
    kp_load_z  = ParameterValue(LaunchConfiguration('kp_load_z'), value_type=float)

    nodes = [
        # 2-drone rigid elevated world: cable_len=1.0, start taut.
        DeclareLaunchArgument('cable_len', default_value='1.0'),
        DeclareLaunchArgument('start_taut', default_value='true'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.05'),
        DeclareLaunchArgument('cable_ff_scale', default_value='1.0'),
        DeclareLaunchArgument('attitude_ff', default_value='true'),
        DeclareLaunchArgument('cable_source', default_value='model'),
        DeclareLaunchArgument('thrust_ratio', default_value='38.0'),
        # closed-loop geometric controller by default for the 2-drone carry.
        DeclareLaunchArgument('planner_mode', default_value='geometric'),
        DeclareLaunchArgument('kp_load_xy', default_value='1.5'),
        DeclareLaunchArgument('kd_load_xy', default_value='0.6'),
        DeclareLaunchArgument('ki_load_xy', default_value='0.3'),
        DeclareLaunchArgument('kp_load_z', default_value='0.0'),
        DeclareLaunchArgument('ff_gate_mode', default_value='airborne'),
        DeclareLaunchArgument('load_traj', default_value='hover'),
        DeclareLaunchArgument('traj_speed', default_value='0.1'),
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
        nodes.append(Node(
            package='controller_quad_load', executable='controller',
            name=f'controller_{i}',
            parameters=[{'drone_id': i, 'reference_source': 'planner',
                         'cable_ff_scale': cable_ff_scale,
                         'attitude_ff': attitude_ff,
                         'cable_source': cable_source,
                         'thrust_ratio': thrust_ratio}],
            output='screen'))

    # ── Central fleet manager (2 drones) ───────────────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': N}], output='screen'))

    # ── Load-state pipeline ────────────────────────────────────────────────
    nodes.append(Node(
        package='controller_load_mpc', executable='load_state', name='load_state',
        parameters=[{'n_drones': N}], output='screen'))

    # ── Centralized planner (n_drones=2) ───────────────────────────────────
    nodes.append(Node(
        package='controller_load_mpc', executable='planner', name='load_planner',
        parameters=[{'n_drones': N,
                      'cable_len': cable_len, 'start_taut': start_taut,
                      'target_z': target_z, 'lift_ramp_vel': lift_ramp_vel,
                      'planner_mode': planner_mode, 'ff_gate_mode': ff_gate_mode,
                      'load_traj': load_traj, 'traj_speed': traj_speed,
                      'traj_distance': traj_distance, 'traj_radius': traj_radius,
                      'kp_load_xy': kp_load_xy, 'kd_load_xy': kd_load_xy,
                      'ki_load_xy': ki_load_xy, 'kp_load_z': kp_load_z}],
        output='screen'))

    return LaunchDescription(nodes)
