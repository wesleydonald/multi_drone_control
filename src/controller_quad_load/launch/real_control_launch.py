"""
real_control_launch.py  —  TERMINAL 2 (control)
-----------------------------------------------
Real-world cable-suspended-payload control layer only, for ANY fleet size
(num_drones): the per-drone cable-aware MPC trackers, the central fleet manager,
and the centralized load-cable OCP planner. Start this AFTER real_io_launch.py
(terminal 1) is up and the MoCap poses are streaming -- the controllers and the
planner block waiting for the first /drone_<id>/motion_capture_state and
/payload/motion_capture_state poses.

This is the hardware twin of the control half of mpc_quad_load_launch.py. It runs
the exact same nodes and carries the same planner/tracker tuning args, but drops
every Gazebo bridge and betaflight_comm (real radios + real MoCap replace them)
and runs on the wall clock (use_sim_time:=false) -- there is no /clock on
hardware. The controllers already pin use_sim_time=False internally; we set it
here too so the planner and fleet manager agree.

  * controller (xN)  — per-drone cable-aware acados MPC (drone_id 0..N-1), reads
    /drone_i/motion_capture_state + /payload/motion_capture_state and the planner
    reference trajectory, publishes /drone_i/ELRSCommand.
  * main             — central fleet manager: owns ARM/TAKEOFF/LAND via
    /fleet/command and the /fleet/step tick.
  * load_planner     — centralized load-cable OCP (Sun et al. 2025). Builds x_init
    from MoCap (load pose/twist + cable directions/rates) and feeds each drone its
    reference trajectory + per-node cable tension accel (the online load-cable OCP).

Run (terminal 2, after terminal 1):
    ros2 launch controller_quad_load real_control_launch.py num_drones:=2
then drive the fleet:
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: LAND}"

HARDWARE NOTES:
  * thrust_ratio defaults to 24 here (real prop/motor kT), not the sim's 45.
    thrust_quad_c:=0.0 by default -- the airborne quadratic-kT schedule is a
    sim-plant fit; on real hardware use the fixed thrust_ratio unless you have
    measured your own a(u)=c*u^2 coefficient.
  * cable_len / load_mass MUST match your physical rig (not the sim SDF).
    load_mass defaults to 0.1 here. Changing it recompiles the acados .so on the
    first launch (the solver cache keys on it), so expect a slower first start.
    LOAD_INERTIA in planner_node.py does NOT scale with load_mass -- scale it by
    hand if the payload's size changed too, not just its mass.
  * Takeoff pace: handover_settle_s (frozen hold) then lift_ramp_vel (climb rate)
    dominate. Defaults here are 0.5 s / 0.20 m/s. The ease-in/ease-out shaping
    (LIFT_SOFT_S, LIFT_SOFT_D in planner_node.py) adds ~1 s that no launch arg
    reaches. Raise lift_ramp_vel further only after a clean slow takeoff.
  * Prove a taut hover (load_traj:=hover) before any circle/fig_8/spin.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def _args():
    return [
        DeclareLaunchArgument('num_drones', default_value='2'),
        # MUST match the physical rig, not the sim SDF.
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        # Physical payload mass (kg). LOAD_INERTIA in planner_node.py is a
        # hardcoded constant and does NOT scale with this -- see the comment there.
        DeclareLaunchArgument('load_mass', default_value='0.1'),
        DeclareLaunchArgument('start_taut', default_value='true'),
        DeclareLaunchArgument('handover_elev_deg', default_value='0.0'),
        # Frozen hold after handover, before the lift ramp starts. Shorter = faster
        # takeoff; too short and the fleet starts climbing before it has settled on
        # the latched config.
        DeclareLaunchArgument('handover_settle_s', default_value='0.5'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        # Climb rate (m/s) -- the dominant term in takeoff duration. The ramp is
        # eased at both ends (LIFT_SOFT_* in planner_node.py), so peak accel stays
        # well under the raw rate. HOLD test: lift_ramp_vel:=0.0 (no lift, just
        # hold the taut config).
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.20'),
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        # Cable compensation. ON is the correct flight config. Zero only for a
        # deliberate A/B (cable_ff_scale:=0.0 cable-blind; attitude_ff:=false level).
        DeclareLaunchArgument('cable_ff_scale', default_value='1.0'),
        DeclareLaunchArgument('attitude_ff', default_value='true'),
        DeclareLaunchArgument('cable_source', default_value='model'),
        DeclareLaunchArgument('payload_rest_z', default_value='0.05'),
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.5'),
        # Real prop/motor thrust accel per unit throttle (kT). ~24 on hardware
        # (vs the sim plant's ~45). Tune to your rig's hover throttle.
        DeclareLaunchArgument('thrust_ratio', default_value='24.0'),
        # Airborne quadratic-kT schedule is a SIM-plant fit -- off by default on
        # hardware. Set to your measured a(u)=c*u^2 coefficient to enable.
        DeclareLaunchArgument('thrust_quad_c', default_value='0.0'),
        # ON: match each drone to the nearest nominal ring slot at the first solve,
        # so you can place the drones ~cable_len out in ANY order (no need to line
        # drone 0 up with +x). Relabels I/O only -- no OCP recompile.
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        # LOAD reference after the lift tops out: 'hover', 'line_x', 'circle',
        # 'fig_8', 'spin' (circle + one full load yaw). circle/fig_8/spin use
        # traj_radius; line_x uses traj_distance; all use traj_speed. Prove hover
        # first, then keep traj_speed slow.
        DeclareLaunchArgument('load_traj', default_value='hover'),
        DeclareLaunchArgument('traj_speed', default_value='0.4'),
        DeclareLaunchArgument('traj_distance', default_value='1.0'),
        DeclareLaunchArgument('traj_radius', default_value='0.5'),
    ]


def launch_setup(context, *args, **kwargs):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)

    # Wall clock on hardware -- there is no /clock. The controllers pin this
    # internally; set it here so the planner + fleet manager agree.
    nodes = [SetParameter(name='use_sim_time', value=False)]

    # ── Per-drone cable-aware MPC trackers ────────────────────────────────────
    for i in range(n):
        nodes.append(Node(
            package='controller_quad_load', executable='controller',
            name=f'controller_{i}',
            parameters=[{'drone_id': i,
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'thrust_ratio': f('thrust_ratio'),
                         'thrust_quad_c': f('thrust_quad_c')}],
            output='screen'))

    # ── Central fleet manager ─────────────────────────────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n}], output='screen'))

    # ── Centralized cable-suspended load planner ──────────────────────────────
    nodes.append(Node(
        package='controller_load_mpc', executable='planner', name='load_planner',
        parameters=[{'num_drones': n,
                     'cable_len': f('cable_len'),
                     'start_taut': b('start_taut'),
                     'load_mass': f('load_mass'),
                     'target_z': f('target_z'),
                     'lift_ramp_vel': f('lift_ramp_vel'),
                     'land_vel': f('land_vel'),
                     'handover_elev_deg': f('handover_elev_deg'),
                     'handover_settle_s': f('handover_settle_s'),
                     'auto_slot_assign': b('auto_slot_assign'),
                     'load_traj': LaunchConfiguration('load_traj'),
                     'traj_speed': f('traj_speed'),
                     'traj_distance': f('traj_distance'),
                     'traj_radius': f('traj_radius')}],
        output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])
