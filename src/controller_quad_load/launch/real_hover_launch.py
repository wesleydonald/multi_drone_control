"""
real_hover_launch.py  --  TERMINAL 2 (control), UNTETHERED HOVER
------------------------------------------------------------------
Fly num_drones drones with NO payload and NO cables: each climbs straight up from where
it sits, hovers at hover_z, and descends on LAND. Same per-drone tracker, fleet manager,
safety envelope and radio path as real_control_launch.py; only the reference source
differs (free_hover instead of the load planner). Start AFTER real_io_launch.py.

Use it to bring up a new airframe on the stack that will later carry the payload, and to
read its hover throttle: with a = kT * throttle and nothing on the hook, the settled
throttle at hover gives kT = 9.81 / throttle for that airframe.

    ros2 launch controller_quad_load real_hover_launch.py num_drones:=3 hover_z:=0.8
    ARM -> TAKEOFF -> (hover) -> LAND       RViz panel or /fleet/command

The tracker never sees /payload/motion_capture_state here and treats the payload as
grounded, i.e. free-flight dynamics (a_cable = 0). tools/preflight.py --planner free_hover
reports the planner read-back as FAIL (no cable geometry to read); everything else applies.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def _args():
    return [
        DeclareLaunchArgument('num_drones', default_value='3'),
        # absolute hover height (m) and the vertical rates of the reference ramp
        DeclareLaunchArgument('hover_z', default_value='0.8'),
        DeclareLaunchArgument('climb_vel', default_value='0.15'),
        DeclareLaunchArgument('land_vel', default_value='0.15'),
        DeclareLaunchArgument('land_tol', default_value='0.10'),
        # tracker args, identical to real_control_launch.py
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.5'),
        DeclareLaunchArgument('thrust_ratio', default_value='24.0'),
        DeclareLaunchArgument('takeoff_thrust_ratio', default_value='0.0'),
        DeclareLaunchArgument('kt_batt_sag_frac', default_value='0.0'),
        DeclareLaunchArgument('kt_batt_v_full', default_value='16.8'),
        DeclareLaunchArgument('kt_batt_v_empty', default_value='14.0'),
        DeclareLaunchArgument('kt_print_period_s', default_value='1.0'),
        DeclareLaunchArgument('terminal_vel_ref', default_value='false'),
        DeclareLaunchArgument('payload_rest_z', default_value='0.05'),
        # mocap watchdog, WALL clock (0.25 = the hardware default). Raise only if a loaded
        # laptop trips it on a healthy link; a real dropout must still disarm.
        DeclareLaunchArgument('pose_timeout_s', default_value='0.25'),
    ]


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)

    nodes = [SetParameter(name='use_sim_time', value=False)]

    for i in range(n):
        nodes.append(Node(
            package='controller_quad_load', executable='controller',
            name=f'controller_{i}',
            parameters=[{'drone_id': i,
                         'terminal_vel_ref': b('terminal_vel_ref'),
                         # no cable: the feedforward is zero on the wire anyway
                         'cable_ff_scale': 1.0,
                         'attitude_ff': True,
                         'cable_source': 'model',
                         'payload_rest_z': f('payload_rest_z'),
                         'pose_timeout_s': f('pose_timeout_s'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'thrust_ratio': f('thrust_ratio'),
                         'takeoff_thrust_ratio': f('takeoff_thrust_ratio'),
                         'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                         'kt_batt_v_full': f('kt_batt_v_full'),
                         'kt_batt_v_empty': f('kt_batt_v_empty'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            output='screen'))

    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n}], output='screen'))

    nodes.append(Node(
        package='controller_quad_load', executable='free_hover', name='free_hover',
        parameters=[{'num_drones': n,
                     'hover_z': f('hover_z'),
                     'climb_vel': f('climb_vel'),
                     'land_vel': f('land_vel'),
                     'land_tol': f('land_tol')}],
        output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])
