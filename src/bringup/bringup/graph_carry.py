"""Node graphs of the carry modes: sim/real mpc (the load planner) and free_hover.

sim mpc (was mpc_quad_load_launch.py): the centralized load planner (mpc_planner) generates
each drone's reference AND its per-node cable tension acceleration; the per-drone tracker
tracks it with the cable force in its prediction model. Per drone: motor and IMU bridges,
the sim Betaflight node and the tracker; then the fleet manager and the planner. The clock,
pose bridges and mocap emulators belong to sim_io_launch.py, which must be up first.
num_drones must match the world: the planner sizes the ring and divides the load by it.
mode:=free_hover replaces the planner with free_hover at target_z (a plant check on a drone
detached from the ring). start_taut true skips the creep: the OCP takes over on its first
tick; on a ground world that asks the no-integrator tracker for the whole formation step
with the cable feedforward still gated off, which is why the floor start creeps.

real mpc (was real_control_launch.py): the same trackers, fleet manager and planner with no
bridge and no sim Betaflight (real radios and mocap replace them), on the wall clock. Start it
after real_io_launch.py is streaming mocap. cable_len / load_mass must be the rig's; a new
load_mass recompiles the planner's solver on the first launch.

real free_hover (was real_hover_launch.py): no payload, no cables: each drone climbs from where
it sits to hover_z and descends on LAND; the settled hover throttle gives that airframe's kT.
"""
from bringup.real_mode import planner_options
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from tracker.thrust_model import resolve_thrust_ratio

PARENT_MODEL = 'lift_system'


def sim_mpc(context, launch_dir, profile_vals):
    # Resolve num_drones to a real int -- this is the whole reason for
    # OpaqueFunction. Everything else can stay a substitution.
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')
    drone_names = [f'x3_drone{i}' for i in range(n)]
    # kT is an operating point of the sim's quadratic motor model (thrust_model.py):
    # 'auto' derives it from load_mass and the fleet size, a number is used verbatim.
    kt, kt_to, kt_note = resolve_thrust_ratio(
        LaunchConfiguration('thrust_ratio').perform(context),
        LaunchConfiguration('takeoff_thrust_ratio').perform(context),
        LaunchConfiguration('load_mass').perform(context), n)

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    sim_offsets = [float(x) for x in
                   LaunchConfiguration('sim_thrust_offset').perform(context).split(',') if x.strip()]
    if len(sim_offsets) not in (1, n):
        raise RuntimeError(f'sim_thrust_offset needs one value or {n}, got {len(sim_offsets)}')
    sim_map = LaunchConfiguration('sim_thrust_map').perform(context).strip().lower()
    # 'auto' is the linear x3 plant's gain; a rig plant needs the rig's measured map typed in.
    if sim_map == 'rig' and (LaunchConfiguration('thrust_ratio').perform(context).strip().lower() == 'auto'
                             or float(LaunchConfiguration('thrust_offset').perform(context)) <= 0.0):
        raise RuntimeError('sim_thrust_map:=rig needs an explicit thrust_ratio and thrust_offset > 0 '
                           '(the rig launch values: 35.2, 0.185)')

    nodes = [SetParameter(name='use_sim_time', value=True),
             LogInfo(msg=f'[launch] {kt_note}; takeoff {kt_to:.2f}')]

    # NOTE: the clock bridge, the drone/payload POSE bridges and the mocap
    # emulators are NOT here -- they belong to sim_io_launch.py, which
    # must be running first. That split is deliberate: it lets you bring up RViz,
    # confirm every drone and the payload are present and publishing, and only
    # then start the controllers. Launching this alone gives the trackers no
    # pose and they will sit waiting for mocap.
    for i, drone_name in enumerate(drone_names):
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
                         'parent_model': PARENT_MODEL,
                         'rate_source': LaunchConfiguration('rate_source'),
                         'imu_topic': f'/drone_{i}/imu',
                         'thrust_map': LaunchConfiguration('sim_thrust_map'),
                         'thrust_offset': sim_offsets[min(i, len(sim_offsets) - 1)],
                         'pack_v0': f('sim_pack_v0')}]))
        # per-drone CABLE-AWARE MPC tracker — tracks the planner reference
        nodes.append(Node(
            package='tracker', executable='tracker',
            name=f'tracker_{i}',
            parameters=[{'drone_id': i,
                         'terminal_vel_ref': b('terminal_vel_ref'),
                         'x0_relax_symmetric': b('x0_relax_symmetric'),
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
                         'w_thr_ref': f('w_thr_ref'),
                         'x0_relax_frac': f('x0_relax_frac'),
                         'ref_time_shift': b('ref_time_shift'),
                         'ground_idle': b('ground_idle'),
                         'cable_accel_cap': f('cable_accel_cap'),
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'pose_timeout_s': ParameterValue(LaunchConfiguration('pose_timeout_s'), value_type=float),
                         'thrust_ratio': kt,
                         'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                         'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                         'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                         'takeoff_thrust_ratio': kt_to,
                         'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                         'kt_batt_v_full': f('kt_batt_v_full'),
                         'kt_batt_v_empty': f('kt_batt_v_empty'),
                         'kt_print_period_s': f('kt_print_period_s'),
                         'throttle_max': f('throttle_max'),
                         'thrust_offset': f('thrust_offset'),
                         'thrust_offset_v_slope': f('thrust_offset_v_slope'),
                         'thrust_v_ref': f('thrust_v_ref')}],
            output='screen'))

    # ── Central fleet manager ──────────────────────────────────────────────
    nodes.append(Node(
        package='fleet_manager', executable='fleet_manager', name='fleet_manager',
        parameters=[{'num_drones': n}], output='screen'))

    reference = LaunchConfiguration('reference').perform(context).strip()
    if reference == 'free_hover':
        nodes.append(Node(
            package='tracker', executable='free_hover', name='free_hover',
            parameters=[{'num_drones': n, 'hover_z': f('target_z')}], output='screen'))
        return nodes
    if reference != 'planner':
        raise RuntimeError(f"reference must be 'planner' or 'free_hover', got {reference!r}")

    # ── Centralized cable-suspended load planner ───────────────────────────
    nodes.append(Node(
        package='mpc_planner', executable='mpc_planner', name='mpc_planner',
        parameters=[{'num_drones': n,
                     'cable_len': f('cable_len'),
                     'attach_azimuths_deg': LaunchConfiguration('attach_azimuths_deg'),
                     'start_taut': b('start_taut'),
                     'load_mass': f('load_mass'),
                     'drone_mass': f('drone_mass'),
                     'target_z': f('target_z'),
                     'lift_ramp_vel': f('lift_ramp_vel'),
                     'z_ki': f('z_ki'),
                     'z_i_max': f('z_i_max'),
                     'z_taut_gate': f('z_taut_gate'),
                     'ff_cap_force': f('ff_cap_force'),
                     'ff_cap_release_s': f('ff_cap_release_s'),
                     'pivot_offset': [0.0, 0.0, float(LaunchConfiguration('pivot_offset_z').perform(context))],
                     'land_vel': f('land_vel'),
                     'handover_elev_deg': f('handover_elev_deg'),
                     'handover_settle_s': f('handover_settle_s'),
                     'creep_vel': f('creep_vel'),
                     'pretension_s': f('pretension_s'),
                     'auto_slot_assign': b('auto_slot_assign'),
                     'measure_rod_len': b('measure_rod_len'),
                     'load_traj': LaunchConfiguration('load_traj'),
                     'traj_speed': f('traj_speed'),
                     'traj_distance': f('traj_distance'),
                     'traj_radius': f('traj_radius'),
                     **{k: float(v) for k, v in
                        ((k, LaunchConfiguration(k).perform(context)) for k in
                         ('attach_radius', 'attach_z', 'rod_tol_frac', 'rod_spread_m', 'z_i_gate'))
                        if v.strip()}}] + planner_options(context),
        output='screen'))

    return nodes


def real_mpc(context, launch_dir, profile_vals):
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
            package='tracker', executable='tracker',
            name=f'tracker_{i}',
            parameters=[{'drone_id': i,
                         'terminal_vel_ref': b('terminal_vel_ref'),
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
                         'w_thr_ref': f('w_thr_ref'),
                         'x0_relax_frac': f('x0_relax_frac'),
                         'ref_time_shift': b('ref_time_shift'),
                         'ground_idle': b('ground_idle'),
                         'cable_accel_cap': f('cable_accel_cap'),
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'thrust_ratio': f('thrust_ratio'),
                         'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                         'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                         'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                         'throttle_max': ParameterValue(LaunchConfiguration('throttle_max'), value_type=float),
                         'thrust_offset': ParameterValue(LaunchConfiguration('thrust_offset'), value_type=float),
                         'thrust_offset_v_slope': ParameterValue(LaunchConfiguration('thrust_offset_v_slope'), value_type=float),
                         'thrust_v_ref': ParameterValue(LaunchConfiguration('thrust_v_ref'), value_type=float),
                         'takeoff_thrust_ratio': f('takeoff_thrust_ratio'),
                         'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                         'kt_batt_v_full': f('kt_batt_v_full'),
                         'kt_batt_v_empty': f('kt_batt_v_empty'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            output='screen'))

    # ── Central fleet manager ─────────────────────────────────────────────────
    nodes.append(Node(
        package='fleet_manager', executable='fleet_manager', name='fleet_manager',
        parameters=[{'num_drones': n,
                     'require_fc_armed': LaunchConfiguration('require_fc_armed').perform(context).lower() == 'true'}],
        output='screen'))

    # ── Centralized cable-suspended load planner ──────────────────────────────
    nodes.append(Node(
        package='mpc_planner', executable='mpc_planner', name='mpc_planner',
        parameters=[{'num_drones': n,
                     'cable_len': f('cable_len'),
                     'attach_azimuths_deg': LaunchConfiguration('attach_azimuths_deg'),
                     'attach_radius': f('attach_radius'),
                     'attach_z': f('attach_z'),
                     'start_taut': b('start_taut'),
                     'load_mass': f('load_mass'),
                     'drone_mass': f('drone_mass'),
                     'target_z': f('target_z'),
                     'lift_ramp_vel': f('lift_ramp_vel'),
                     'z_ki': f('z_ki'),
                     'z_i_max': f('z_i_max'),
                     'z_i_gate': f('z_i_gate'),
                     'z_taut_gate': f('z_taut_gate'),
                     'pivot_offset': [0.0, 0.0, float(LaunchConfiguration('pivot_offset_z').perform(context))],
                     'solve_budget_s': f('solve_budget_s'),
                     'land_vel': f('land_vel'),
                     'handover_elev_deg': f('handover_elev_deg'),
                     'handover_settle_s': f('handover_settle_s'),
                     'creep_vel': f('creep_vel'),
                     'pretension_s': f('pretension_s'),
                     'ff_cap_release_s': f('ff_cap_release_s'),
                     'rod_tol_frac': f('rod_tol_frac'),
                     'rod_spread_m': f('rod_spread_m'),
                     'auto_slot_assign': b('auto_slot_assign'),
                     'measure_rod_len': b('measure_rod_len'),
                     'load_traj': LaunchConfiguration('load_traj'),
                     'traj_speed': f('traj_speed'),
                     'traj_distance': f('traj_distance'),
                     'traj_radius': f('traj_radius')}] + planner_options(context),
        output='screen'))

    return nodes


def real_free_hover(context, launch_dir, profile_vals):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)

    nodes = [SetParameter(name='use_sim_time', value=False)]

    for i in range(n):
        nodes.append(Node(
            package='tracker', executable='tracker',
            name=f'tracker_{i}',
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
                         'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                         'thrust_offset': f('thrust_offset'),
                         'thrust_offset_v_slope': f('thrust_offset_v_slope'),
                         'thrust_v_ref': f('thrust_v_ref'),
                         'throttle_max': f('throttle_max'),
                         'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                         'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                         'takeoff_thrust_ratio': f('takeoff_thrust_ratio'),
                         'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                         'kt_batt_v_full': f('kt_batt_v_full'),
                         'kt_batt_v_empty': f('kt_batt_v_empty'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            output='screen'))

    nodes.append(Node(
        package='fleet_manager', executable='fleet_manager', name='fleet_manager',
        parameters=[{'num_drones': n,
                     'require_fc_armed': LaunchConfiguration('require_fc_armed').perform(context).lower() == 'true'}],
        output='screen'))

    nodes.append(Node(
        package='tracker', executable='free_hover', name='free_hover',
        parameters=[{'num_drones': n,
                     'hover_z': f('hover_z'),
                     'climb_vel': f('climb_vel'),
                     'land_vel': f('land_vel'),
                     'land_tol': f('land_tol')}],
        output='screen'))

    return nodes
