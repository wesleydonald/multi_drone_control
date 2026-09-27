"""
m2_bench_io_launch.py
---------------------
Sim interface for the M2 hand-over bench (G5, GOALS step 3): Tejen's four X3s already
welded to the 0.86 kg ring at his success-hold pose, hanging on detachable hangers
(tools/sim_test/make_m2_bench_world.py). Starts exactly his per-drone sim plumbing from
tejen_mission/launch/m2d_four_drone_sequential.launch.py -- /clock bridge (unthrottled:
never throttle /clock in his world), ring pose bridge, magnet joint command bridge, and
per drone the pose+motor bridge, tejen_motion_capture_emulator and
tejen_betaflight_communication -- plus the /bench/hanger_i bridges. Nothing of his
mission (planners, backends, capture managers, observers, supervisor, MPCs) and not his
pendulum_state_publisher (nothing of ours reads it).

    ros2 launch controller_quad_load m2_bench_io_launch.py                # starts gz too
    ros2 launch controller_quad_load m2_bench_io_launch.py start_gz:=false  # run_experiment owns gz

Then dissipative_launch.py partner_m2:=true sim_interface:=false ... (configs/experiments/
m2_bench.yaml has the args), ARM, /fleet/handover, TAKEOFF, release the hangers:
    ros2 topic pub --once /bench/hanger_0/detach std_msgs/msg/Empty '{}'   # (0..3)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__)))))
BENCH_WORLD = os.path.join(REPO, 'simulation_assets', 'tejen', 'bench_m2', 'm2_bench_world.sdf')
N_DRONES = 4


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ('1', 'true', 'yes')


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n != N_DRONES:
        raise RuntimeError(f'the M2 bench world has {N_DRONES} drones, got num_drones:={n}')
    world = LaunchConfiguration('world').perform(context)
    rate_ki = float(LaunchConfiguration('rate_ki').perform(context))
    actions = []

    if _truthy(context, 'start_gz'):
        if not os.path.exists(world):
            raise RuntimeError(f'{world} missing: run tools/sim_test/make_m2_bench_world.py')
        gui = _truthy(context, 'gui')
        actions.append(ExecuteProcess(
            cmd=['gz', 'sim', '-r', '-v', '2', world] if gui else
                ['gz', 'sim', '-s', '-r', '-v', '2', world],
            name='m2_bench_gazebo', output='screen', sigterm_timeout='3', sigkill_timeout='2'))

    # his m2d launch (m2d_four_drone_sequential.launch.py) l.279-294
    actions.append(TimerAction(period=0.50, actions=[Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='m2d_clock_bridge',
        output='screen', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])]))
    tejen_share = get_package_share_directory('tejen_mission')
    hanger_args = []
    for i in range(n):
        hanger_args += [f'/bench/hanger_{i}/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                        f'/bench/hanger_{i}/state@std_msgs/msg/String[gz.msgs.StringMsg']
    actions.append(TimerAction(period=0.75, actions=[
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='m2d_ring_pose_bridge',
             output='screen',
             arguments=['/model/payload_model/pose@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='m2d_joint_command_bridge',
             output='screen',
             parameters=[{'config_file': os.path.join(tejen_share, 'config', 'm2c_gz_joint_bridge.yaml')}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='m2_bench_hanger_bridge',
             output='screen', arguments=hanger_args),
    ]))

    # his m2d launch l.308-348 (platform_nodes without the pendulum publisher), started at 1.0 s as there
    platform = []
    for i in range(n):
        ns = f'drone_{i}'
        pose_topic = f'/model/x3_{i}/pose'
        motor_topic = f'/{ns}/gazebo/command/motor_speed'
        platform += [
            Node(package='ros_gz_bridge', executable='parameter_bridge',
                 name=f'm2d_pose_motor_bridge_{i}', output='screen',
                 arguments=[f'{pose_topic}@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V',
                            f'{motor_topic}@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']),
            Node(package='simulation_communication', executable='tejen_motion_capture_emulator',
                 namespace=ns, name='motion_capture_emulator', output='screen',
                 parameters=[{'use_sim_time': True, 'target_object_id': 7,
                              'pose_topic': pose_topic,
                              'motion_capture_state_topic': 'motion_capture_state',
                              'rviz_pose_topic': 'rviz_pose',
                              'enable_orientation_bias': False}]),
            Node(package='simulation_communication', executable='tejen_betaflight_communication',
                 namespace=ns, name='betaflight_communication', output='screen',
                 parameters=[{'use_sim_time': True, 'pose_topic': pose_topic, 'pose_index': 7,
                              'elrs_command_topic': 'ELRSCommand',
                              'motor_command_topic': motor_topic,
                              'publish_telemetry': True, 'telemetry_topic': 'telemetry',
                              'rates_d_val': 100.0, 'rates_f_val': 100.0, 'rates_g_val': 0.0,
                              'rate_ki': rate_ki}]),
        ]
    actions.append(TimerAction(period=1.00, actions=platform))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value=str(N_DRONES)),
        # run_experiment passes these to every io launch; the bench has no newcomer and no RViz
        DeclareLaunchArgument('attach', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('start_gz', default_value='true'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('world', default_value=BENCH_WORLD),
        # his launch's default: M2_RATE_KI, else SIM_RATE_KI, else 5.0
        DeclareLaunchArgument('rate_ki', default_value=os.environ.get(
            'M2_RATE_KI', os.environ.get('SIM_RATE_KI', '5.0'))),
        OpaqueFunction(function=launch_setup),
    ])
