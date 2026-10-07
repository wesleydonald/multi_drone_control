"""Node graphs of the detach and network modes, and the M2 rig mode.

sim dissipative (was dissipative_launch.py): the carry stack with the dissipative planner in
place of mpc_planner. Takeoff is the same LoadPlanner OCP; at /fleet/detach <k> the fleet is
resized (reconfig_mode ocp: the OCP is rebuilt for n-1 and keeps flying; network: the
spring-damper network takes the fleet) and the trackers are unchanged (same reference wire
format). Per drone a detach bridge (/drone_k/detach -> the world's DetachableJoint): run a
--detachable world. partner_m2: our stack on top of Tejen's M2 world after his drones joined:
his stacks own /drone_i command and arming names, so ours move under /ours, and each drone gets
an ELRS mux that forwards his MPC until /fleet/handover, then ours. sim_interface:=false leaves
the Gazebo bridges and Betaflight nodes to his launch.

real m2 (was dissipative_launch.py real:=true; the rig flies this for M2, partner_m2:=true):
one launch that includes real_io_launch.py itself. No Gazebo-facing node, the wall clock, a
typed kT, and bringup/real_mode.py's rig watchdogs and creep start in place of the sim
defaults (a typed value kept, a looser watchdog refused). Magnets: each radio's String latch
/drone_<i>/magnet on channel magnet_channel; the planner (detach_magnet) latches every tether ON
at ARM and OFF at its detach. real_io:=false when Tejen's IRL stack owns the radios and mocap.

sim network (was dissipative_only_launch.py): the dissipative planner hands the fleet to the
network once the OCP lift tops out (auto_network_handover), so the whole flight after the
lift is flown on the network. Nothing detaches; no detach bridges.

real dissipative (was real_dissipative_launch.py): the two-terminal rig carry/detach of the R
ladder: trackers, fleet manager and the dissipative planner after real_io_launch.py, with
real_control's rig values (decisions 2026-10-03). A detach resizes the OCP and releases that
drone's magnet on the same tick (reconfig_mode ocp, detach_magnet true).
"""
from bringup.real_mode import (apply_rig_values, map_and_geometry, planner_options, rig_io,
                               rig_magnet_latches, rig_thrust_ratio, tracker_drop_options, truthy)
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from tracker.thrust_model import resolve_thrust_ratio

PARENT_MODEL = 'lift_system'


def dissipative(context, launch_dir, profile_vals):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')
    drone_names = [f'x3_drone{i}' for i in range(n)]
    real = truthy(context, 'real')
    if real:
        kt, kt_to, kt_note = rig_thrust_ratio(
            LaunchConfiguration('thrust_ratio').perform(context),
            LaunchConfiguration('takeoff_thrust_ratio').perform(context))
        rig_note = apply_rig_values(context, profile_vals)
    else:
        # kT is an operating point of the sim's quadratic motor model (thrust_model.py):
        # 'auto' derives it from load_mass and the fleet size, a number is used verbatim.
        kt, kt_to, kt_note = resolve_thrust_ratio(
            LaunchConfiguration('thrust_ratio').perform(context),
            LaunchConfiguration('takeoff_thrust_ratio').perform(context),
            LaunchConfiguration('load_mass').perform(context), n)

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    i_ = lambda name: ParameterValue(LaunchConfiguration(name), value_type=int)

    nodes = [SetParameter(name='use_sim_time', value=not real),
             LogInfo(msg=f'[launch] {kt_note}; takeoff {kt_to:.2f}')]
    if real:
        nodes.append(LogInfo(msg=rig_note))
    twin_tracker, twin_bf, twin_planner = map_and_geometry(context, n, real)

    # partner_m2 (multi_drone_control 2026-09-26): our stack on top of Tejen's M2 world
    # after his four drones have joined the ring. His per-drone stacks own the /drone_i
    # command/arming names, so ours move under /ours; each drone gets an ELRS mux that
    # forwards his MPC (/drone_i/ELRSCommand_tejen) until /fleet/handover, then ours.
    # the rig has no sim interface: its controllers take the no-bridge branch below
    sim_interface = (not real) and truthy(context, 'sim_interface')
    partner_m2 = LaunchConfiguration('partner_m2').perform(context).lower() in ('1', 'true', 'yes')
    part = LaunchConfiguration('partner_m2_part').perform(context) if partner_m2 else 'all'
    want_muxes = part in ('all', 'muxes')
    want_ctrl = part in ('all', 'controllers')
    if real and truthy(context, 'real_io') and want_muxes:
        # radios with the muxes: they carry whichever stream the mux forwards. No DETACH
        # row: M2 has no detach step (a /fleet/detach still releases that drone's magnet)
        nodes += rig_io(launch_dir, n,
                        rig_magnet_latches(
                            context, [LaunchConfiguration('magnet_initial').perform(context)] * n))

    def ours(i):
        return [(f'/drone_{i}/arming_service', f'/ours/drone_{i}/arming_service'),
                (f'/drone_{i}/arming_state_feedback', f'/ours/drone_{i}/arming_state_feedback'),
                (f'/drone_{i}/command', f'/ours/drone_{i}/command'),
                (f'/drone_{i}/ELRSCommand', f'/drone_{i}/ELRSCommand_diss')] if partner_m2 else []

    for i, drone_name in enumerate(drone_names):
        if partner_m2 and want_muxes:
            nodes.append(Node(
                package='drone_magnet', executable='elrs_mux', name=f'elrs_mux_{i}',
                parameters=[{'drone_id': i, 'latch': True, 'approach_stream': True,
                             'handoff_topic': '/fleet/handover',
                             **({'magnet_channel': i_('magnet_channel'),
                                 'magnet_command_topic': ''} if real else {})}],
                output='screen'))
        if not want_ctrl:
            continue
        if not sim_interface:
            nodes.append(Node(
                package='tracker', executable='tracker',
                name=f'tracker_{i}', remappings=ours(i),
                parameters=[{'drone_id': i,
                             'terminal_vel_ref': b('terminal_vel_ref'),
                             'cable_ff_scale': f('cable_ff_scale'),
                             'attitude_ff': b('attitude_ff'),
                             'cable_source': LaunchConfiguration('cable_source'),
                             'w_thr_ref': ParameterValue(LaunchConfiguration('w_thr_ref'), value_type=float),
                             'x0_relax_frac': ParameterValue(LaunchConfiguration('x0_relax_frac'), value_type=float),
                             'ref_time_shift': ParameterValue(LaunchConfiguration('ref_time_shift'), value_type=bool),
                         **tracker_drop_options(context),
                         'ground_idle': ParameterValue(LaunchConfiguration('ground_idle'), value_type=bool),
                             'cable_accel_cap': ParameterValue(LaunchConfiguration('cable_accel_cap'), value_type=float),
                             'payload_rest_z': f('payload_rest_z'),
                             'takeoff_spool_s': f('takeoff_spool_s'),
                             'pose_timeout_s': ParameterValue(LaunchConfiguration('pose_timeout_s'), value_type=float),
                             'safety_ref_timeout_s': f('safety_ref_timeout_s'),
                             'thrust_ratio': kt,
                             'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                             'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                             'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                             'takeoff_thrust_ratio': kt_to,
                             'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                             'kt_batt_v_full': f('kt_batt_v_full'),
                             'kt_batt_v_empty': f('kt_batt_v_empty'),
                             'kt_print_period_s': f('kt_print_period_s'),
                             **twin_tracker}],
                output='screen'))
            continue
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
        # DETACH trigger bridge: ROS std_msgs/Empty -> gz.msgs.Empty on the
        # DetachableJoint's detach_topic (/drone_i/detach in the --detachable world).
        # ']' = ROS-to-GZ only. If the installed bridge uses the 'ignition.msgs'
        # namespace (as the motor bridge above does), change gz.msgs.Empty to
        # ignition.msgs.Empty.
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            name=f'detach_bridge_{i}',
            arguments=[f'/drone_{i}/detach@std_msgs/msg/Empty]gz.msgs.Empty']))
        nodes.append(Node(
            package='simulation_communication', executable='payload_betaflight_comm',
            name=f'bf_comm_{i}',
            parameters=[{'drone_id': i, 'drone_name': drone_name,
                         'parent_model': PARENT_MODEL,
                         'rate_source': LaunchConfiguration('rate_source'),
                         'imu_topic': f'/drone_{i}/imu',
                         **{k: (v[i] if isinstance(v, list) else v) for k, v in twin_bf.items()}}]))
        nodes.append(Node(
            package='tracker', executable='tracker',
            name=f'tracker_{i}',
            parameters=[{'drone_id': i,
                         'terminal_vel_ref': b('terminal_vel_ref'),
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
                         'w_thr_ref': ParameterValue(LaunchConfiguration('w_thr_ref'), value_type=float),
                         'x0_relax_frac': ParameterValue(LaunchConfiguration('x0_relax_frac'), value_type=float),
                         'ref_time_shift': ParameterValue(LaunchConfiguration('ref_time_shift'), value_type=bool),
                         **tracker_drop_options(context),
                         'ground_idle': ParameterValue(LaunchConfiguration('ground_idle'), value_type=bool),
                         'cable_accel_cap': ParameterValue(LaunchConfiguration('cable_accel_cap'), value_type=float),
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'pose_timeout_s': ParameterValue(LaunchConfiguration('pose_timeout_s'), value_type=float),
                             'safety_ref_timeout_s': f('safety_ref_timeout_s'),
                         'thrust_ratio': kt,
                         'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                         'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                         'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                         'takeoff_thrust_ratio': kt_to,
                         'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                         'kt_batt_v_full': f('kt_batt_v_full'),
                         'kt_batt_v_empty': f('kt_batt_v_empty'),
                         'kt_print_period_s': f('kt_print_period_s'),
                         **twin_tracker}],
            output='screen'))

    if not want_ctrl:
        return nodes

    # ── Central fleet manager ──────────────────────────────────────────────
    if partner_m2 and not real:
        # his ring (M2A fixture, made dynamic) -> our payload mocap
        nodes.append(Node(
            package='simulation_communication', executable='tejen_motion_capture_emulator',
            name='payload_mocap', parameters=[{
                'pose_topic': '/model/payload_model/pose', 'target_object_id': 1,
                'motion_capture_state_topic': '/payload/motion_capture_state',
                'rviz_pose_topic': '/payload/rviz_pose'}], output='screen'))
    fleet_remaps = [r for i in range(n) for r in ours(i)]
    nodes.append(Node(
        package='fleet_manager', executable='fleet_manager', name='fleet_manager',
        remappings=fleet_remaps, parameters=[{'num_drones': n}], output='screen'))

    # ── Decentralized dissipative reference generator ──────────────────────
    nodes.append(Node(
        package='dissipative_planner', executable='dissipative_planner', name='dissipative_planner',
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
                     'land_vel': f('land_vel'),
                     'handover_elev_deg': f('handover_elev_deg'),
                     'handover_settle_s': f('handover_settle_s'),
                     'creep_vel': f('creep_vel'),
                     'auto_slot_assign': b('auto_slot_assign'),
                     'measure_rod_len': b('measure_rod_len'),
                     'load_traj': LaunchConfiguration('load_traj'),
                     'traj_speed': f('traj_speed'),
                     'traj_distance': f('traj_distance'),
                     'traj_radius': f('traj_radius'),
                     'net_horizon_preview': b('net_horizon_preview'),
                     'net_traj_lean': b('net_traj_lean'),
                     'diss_k_pay': f('diss_k_pay'),
                     'diss_k_anchor': f('diss_k_anchor'),
                     'diss_c': f('diss_c'),
                     'diss_k_ring': f('diss_k_ring'),
                     'diss_k_slot': f('diss_k_slot'),
                     'diss_node_mass': f('diss_node_mass'),
                     'diss_substeps': i_('diss_substeps'),
                     'diss_elev_deg': f('diss_elev_deg'),
                     'reconfig_mode': LaunchConfiguration('reconfig_mode'),
                     'reconfig_hold_s': f('reconfig_hold_s'),
                     'min_survivors': i_('min_survivors'),
                     'net_land_z': f('net_land_z'),
                     'attach_z': f('attach_z'),
                     'airborne_start': b('airborne_start'),
                     'cable_elev_deg': f('cable_elev_deg'),
                     # rig: tether latches ON at ARM, OFF at their detach
                     **({'detach_magnet': True} if real else {}),
                     **twin_planner}] + planner_options(context),
        output='screen'))

    return nodes



def sim_network(context, launch_dir, profile_vals):
    # Resolve num_drones to a real int -- a LaunchConfiguration is an unresolved
    # substitution at generate_launch_description() time, so the per-drone loop has to
    # happen inside an OpaqueFunction.
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
    i_ = lambda name: ParameterValue(LaunchConfiguration(name), value_type=int)

    nodes = [SetParameter(name='use_sim_time', value=True),
             LogInfo(msg=f'[launch] {kt_note}; takeoff {kt_to:.2f}')]

    # NOTE: the clock bridge, the drone/payload POSE bridges and the mocap emulators are
    # NOT here -- they belong to sim_io_launch.py, which must be running first.
    # Launching this alone gives the trackers no pose and they sit waiting for mocap.
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
        # No detach bridge: nothing detaches in this stack, so the world does not need to
        # be the --detachable variant.
        nodes.append(Node(
            package='simulation_communication', executable='payload_betaflight_comm',
            name=f'bf_comm_{i}',
            parameters=[{'drone_id': i, 'drone_name': drone_name,
                         'parent_model': PARENT_MODEL,
                         'rate_source': LaunchConfiguration('rate_source'),
                         'imu_topic': f'/drone_{i}/imu'}]))
        # per-drone CABLE-AWARE MPC tracker. UNCHANGED from the other stacks: it consumes
        # the same reference wire format whether the OCP or the network produced it.
        nodes.append(Node(
            package='tracker', executable='tracker',
            name=f'tracker_{i}',
            parameters=[{'drone_id': i,
                         'terminal_vel_ref': b('terminal_vel_ref'),
                         'control_mode': LaunchConfiguration('control_mode'),
                         'vel_kp_pos': f('vel_kp_pos'),
                         'vel_kv': f('vel_kv'),
                         'vel_ki': f('vel_ki'),
                         'vel_k_att': f('vel_k_att'),
                         'vel_swing_k': f('vel_swing_k'),
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
                         'w_thr_ref': ParameterValue(LaunchConfiguration('w_thr_ref'), value_type=float),
                         'x0_relax_frac': ParameterValue(LaunchConfiguration('x0_relax_frac'), value_type=float),
                         'ref_time_shift': ParameterValue(LaunchConfiguration('ref_time_shift'), value_type=bool),
                         **tracker_drop_options(context),
                         'ground_idle': ParameterValue(LaunchConfiguration('ground_idle'), value_type=bool),
                         'cable_accel_cap': ParameterValue(LaunchConfiguration('cable_accel_cap'), value_type=float),
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'thrust_ratio': kt,
                         'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                         'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                         'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                         'takeoff_thrust_ratio': kt_to,
                         'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                         'kt_batt_v_full': f('kt_batt_v_full'),
                         'kt_batt_v_empty': f('kt_batt_v_empty'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            output='screen'))

    # ── Central fleet manager ──────────────────────────────────────────────
    nodes.append(Node(
        package='fleet_manager', executable='fleet_manager', name='fleet_manager',
        parameters=[{'num_drones': n}], output='screen'))

    # ── Dissipative reference generator (OCP takeoff -> network for the rest) ──
    # reserved_attach is left at its 0 default: no mid-flight newcomer in this stack.
    nodes.append(Node(
        package='dissipative_planner', executable='dissipative_planner',
        name='dissipative_planner',
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
                     'land_vel': f('land_vel'),
                     'handover_elev_deg': f('handover_elev_deg'),
                     'handover_settle_s': f('handover_settle_s'),
                     'creep_vel': f('creep_vel'),
                     'auto_slot_assign': b('auto_slot_assign'),
                     'measure_rod_len': b('measure_rod_len'),
                     'load_traj': LaunchConfiguration('load_traj'),
                     'traj_speed': f('traj_speed'),
                     'traj_distance': f('traj_distance'),
                     'traj_radius': f('traj_radius'),
                     'auto_network_handover': b('auto_network_handover'),
                     'auto_handover_settle_s': f('auto_handover_settle_s'),
                     'net_horizon_preview': b('net_horizon_preview'),
                     'net_traj_lean': b('net_traj_lean'),
                     'diss_k_pay': f('diss_k_pay'),
                     'diss_k_anchor': f('diss_k_anchor'),
                     'diss_c': f('diss_c'),
                     'diss_k_ring': f('diss_k_ring'),
                     'diss_k_slot': f('diss_k_slot'),
                     'diss_node_mass': f('diss_node_mass'),
                     'diss_substeps': i_('diss_substeps'),
                     'reconfig_mode': LaunchConfiguration('reconfig_mode'),
                     'reconfig_hold_s': f('reconfig_hold_s'),
                     'diss_elev_deg': f('diss_elev_deg'),
                     'diss_ki_load': f('diss_ki_load'),
                     'diss_a_i_load_max': f('diss_a_i_load_max'),
                     'diss_trim_share_weighted': b('diss_trim_share_weighted'),
                     'net_land_z': f('net_land_z')}] + planner_options(context),
        output='screen'))

    return nodes


def real_dissipative(context, launch_dir, profile_vals):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    i_ = lambda name: ParameterValue(LaunchConfiguration(name), value_type=int)

    # Wall clock on hardware -- there is no /clock. The controllers pin this
    # internally; set it here so the dissipative node and fleet manager agree.
    nodes = [SetParameter(name='use_sim_time', value=False)]

    # ── Per-drone cable-aware MPC trackers (identical to the OCP stack) ───────
    for i in range(n):
        nodes.append(Node(
            package='tracker', executable='tracker',
            name=f'tracker_{i}',
            parameters=[{'drone_id': i,
                         'terminal_vel_ref': b('terminal_vel_ref'),
                         'control_mode': LaunchConfiguration('control_mode'),
                         'vel_kp_pos': f('vel_kp_pos'),
                         'vel_kv': f('vel_kv'),
                         'vel_ki': f('vel_ki'),
                         'vel_k_att': f('vel_k_att'),
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
                         'w_thr_ref': ParameterValue(LaunchConfiguration('w_thr_ref'), value_type=float),
                         'x0_relax_frac': ParameterValue(LaunchConfiguration('x0_relax_frac'), value_type=float),
                         'ref_time_shift': ParameterValue(LaunchConfiguration('ref_time_shift'), value_type=bool),
                         **tracker_drop_options(context),
                         'ground_idle': ParameterValue(LaunchConfiguration('ground_idle'), value_type=bool),
                         'cable_accel_cap': ParameterValue(LaunchConfiguration('cable_accel_cap'), value_type=float),
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'thrust_ratio': f('thrust_ratio'),
                         'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                         'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                         'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                         'throttle_max': f('throttle_max'),
                         'thrust_offset': f('thrust_offset'),
                         'thrust_offset_v_slope': f('thrust_offset_v_slope'),
                         'thrust_v_ref': f('thrust_v_ref'),
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

    # ── Decentralized dissipative reference generator ─────────────────────────
    # Replaces mpc_planner/planner. OCP takeoff until /fleet/detach, then
    # the spring-damper network. reserved_attach is left at its default 0, so no
    # attach topics are created.
    nodes.append(Node(
        package='dissipative_planner', executable='dissipative_planner',
        name='dissipative_planner',
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
                     'traj_radius': f('traj_radius'),
                     'auto_network_handover': b('auto_network_handover'),
                     'auto_handover_settle_s': f('auto_handover_settle_s'),
                     'diss_ki_load': f('diss_ki_load'),
                     'diss_a_i_load_max': f('diss_a_i_load_max'),
                     'diss_trim_share_weighted': b('diss_trim_share_weighted'),
                     'net_horizon_preview': b('net_horizon_preview'),
                     'net_traj_lean': b('net_traj_lean'),
                     'diss_k_pay': f('diss_k_pay'),
                     'diss_k_anchor': f('diss_k_anchor'),
                     'diss_c': f('diss_c'),
                     'diss_k_ring': f('diss_k_ring'),
                     'diss_k_slot': f('diss_k_slot'),
                     'diss_node_mass': f('diss_node_mass'),
                     'diss_substeps': i_('diss_substeps'),
                     'diss_elev_deg': f('diss_elev_deg'),
                     'reconfig_mode': LaunchConfiguration('reconfig_mode'),
                     'reconfig_hold_s': f('reconfig_hold_s'),
                     'detach_magnet': b('detach_magnet'),
                     'min_survivors': i_('min_survivors'),
                     'net_land_z': f('net_land_z')}] + planner_options(context),
        output='screen'))

    return nodes
