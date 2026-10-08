"""Node graph of the attach mode (sim and real).

Three tethered drones carry the ring on the
dissipative planner, and a fourth free drone (x3_drone3, a 0.5 m magnet arm) flies to the ring,
welds on and joins, so the trajectory continues with four (the reverse of the detach).

sim attach (was three_attach_launch.py): world three_attach*.sdf; sim_io_launch.py attach:=true
first. Adds the tethered trackers, the fleet manager (managing the newcomer too), the
dissipative planner (reserved_attach 1), and the whole drone-3 chain: its own bridges and
mocap (it is standalone, not nested in lift_system), the collaborator's approach MPC (or our
tracker flying the approach, enable_approach_mpc:=false), the attach target, the join planner,
the magnet manager and tip publisher, our drone-3 tracker and the ELRS mux that forwards the
approach until the weld and ours after it. /magnet/object_attached flips the mux and folds
drone 3 into the planner. sil:=true brings up only the controllers (tools/sil_bench.py supplies
mocap, IMU, /clock and a scripted weld). partner:=true lets Tejen's mission fly the newcomer
(start tejen_mission partner_mission.launch.py); partner_attached:=true starts it welded as the
fourth carrier, then it leaves and rejoins.

real attach (was three_attach_launch.py real:=true; the rig flies this for M1): one launch that
includes real_io_launch.py itself, no Gazebo-facing node, the wall clock, a typed kT and the rig
watchdogs and creep start of bringup/real_mode.py (a typed value kept). The magnet manager welds
on the mocap tip body (/magnet_tip_pose; speed gate 0.05 m/s, dwell 0.15 s). Magnets: tethers
latch magnet_initial from boot and the planner re-latches them ON at ARM, OFF at their detach;
the newcomer latches attach_magnet_initial ('auto': ON for the welded partner start, else OFF)
and then the magnet manager owns it. weld_radius:=0 is a dry approach: no weld, the newcomer's
magnet stays OFF (a typed ON is refused). Needs on the rig: the tip body in the mocap map, the
four drones' and the ring's body ids, the prebuilt solver for the weighed masses.
"""
from bringup.real_mode import (apply_rig_values, map_and_geometry, planner_options, rig_io,
                               rig_magnet_latches, rig_thrust_ratio, truthy)
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from tracker.thrust_model import resolve_thrust_ratio

PARENT_MODEL = 'lift_system'
ATTACH_DRONE_ID = 3
ATTACH_DRONE_NAME = 'x3_drone3'


def _x0_rate_imu_drones(context, n, real):
    """Tethered drone ids whose tracker uses x0_rate_source imu (x0_rate_source_drones)."""
    typed = LaunchConfiguration('x0_rate_source_drones').perform(context).strip()
    if not typed:
        return set()
    if real:
        raise RuntimeError('the rig refuses x0_rate_source_drones: the gyro x0 rate is a '
                           'sim oracle')
    ids = {int(t) for t in typed.split(',') if t.strip()}
    if not ids <= set(range(n)):
        raise RuntimeError(f'x0_rate_source_drones {sorted(ids)}: only tethered drones '
                           f'0..{n - 1}')
    return ids


def attach(context, launch_dir, profile_vals):
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')
    drone_names = [f'x3_drone{i}' for i in range(n)]
    sil = truthy(context, 'sil')
    real = truthy(context, 'real')
    if real and sil:
        raise RuntimeError('real:=true and sil:=true are exclusive')
    if real:
        kt, kt_to, kt_note = rig_thrust_ratio(
            LaunchConfiguration('thrust_ratio').perform(context),
            LaunchConfiguration('takeoff_thrust_ratio').perform(context))
        kt_solo = kt
        rig_note = apply_rig_values(context, profile_vals)
    else:
        # kT is an operating point of the sim's quadratic motor model (thrust_model.py):
        # 'auto' derives it from load_mass and the fleet size, a number is used verbatim.
        kt, kt_to, kt_note = resolve_thrust_ratio(
            LaunchConfiguration('thrust_ratio').perform(context),
            LaunchConfiguration('takeoff_thrust_ratio').perform(context),
            LaunchConfiguration('load_mass').perform(context), n)
        kt_solo = resolve_thrust_ratio(LaunchConfiguration('thrust_ratio').perform(context),
                                       '0', 0.0, 1)[0]

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)
    i_ = lambda name: ParameterValue(LaunchConfiguration(name), value_type=int)

    nodes = [SetParameter(name='use_sim_time', value=not real),
             LogInfo(msg=f'[launch] {kt_note}; takeoff {kt_to:.2f}')]
    if real:
        nodes.append(LogInfo(msg=rig_note))

    # SIL: controllers only. The bench is the simulator, so every Gazebo-facing node
    # below is skipped. See the `sil` launch argument. The rig has none of them either.
    sim_io = not sil and not real
    # the collaborator's approach MPC flies the newcomer to the weld; without it the
    # dissipative node flies the approach on the newcomer's own tracker (approach.py)
    enable_approach_mpc = (not sil) and (
        LaunchConfiguration('enable_approach_mpc').perform(context).lower()
        in ('1', 'true', 'yes'))
    # partner: Tejen's mission (tejen_mission partner_mission.launch.py) flies the newcomer;
    # the mux forwards his stream on /drone_3/ELRSCommand_tejen until the ring weld
    partner = (not sil) and (
        LaunchConfiguration('partner').perform(context).lower() in ('1', 'true', 'yes'))
    if partner:
        enable_approach_mpc = False
    # partner demo start: drone 3 starts WELDED as the fourth carrier, detaches, is
    # stepped out and released to the partner's mission, then rejoins
    partner_attached = partner and (
        LaunchConfiguration('partner_attached').perform(context).lower() in ('1', 'true', 'yes'))

    x0_imu = _x0_rate_imu_drones(context, n, real)
    # rig: the carry's thrust map and geometry; the sim attach graph passes neither
    map_tracker, _, map_planner = map_and_geometry(context, n, real) if real else ({}, {}, {})

    # ── 3 TETHERED drones: bridges + betaflight comm + our tracker (as dissipative_launch) ──
    for i, drone_name in enumerate(drone_names):
        if sim_io:
            nodes.append(Node(
                package='ros_gz_bridge', executable='parameter_bridge', name=f'motor_bridge_{i}',
                arguments=[f'/{drone_name}/gazebo/command/motor_speed'
                           f'@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']))
            nodes.append(Node(
                package='ros_gz_bridge', executable='parameter_bridge', name=f'imu_bridge_{i}',
                arguments=[f'/{drone_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
                remappings=[(f'/{drone_name}/imu', f'/drone_{i}/imu')]))
            nodes.append(Node(
                package='ros_gz_bridge', executable='parameter_bridge', name=f'detach_bridge_{i}',
                arguments=[f'/drone_{i}/detach@std_msgs/msg/Empty]gz.msgs.Empty']))
            nodes.append(Node(
                package='simulation_communication', executable='payload_betaflight_comm',
                name=f'bf_comm_{i}',
                parameters=[{'drone_id': i, 'drone_name': drone_name,
                             'parent_model': PARENT_MODEL,
                             'rate_source': LaunchConfiguration('rate_source'),
                             'imu_topic': f'/drone_{i}/imu'}]))
        nodes.append(Node(
            package='tracker', executable='tracker', name=f'tracker_{i}',
            parameters=[{'drone_id': i,
                         'terminal_vel_ref': b('terminal_vel_ref'),
                         'x0_relax_symmetric': b('x0_relax_symmetric'),
                         **({'x0_rate_source': 'imu'} if i in x0_imu else {}),
                         'control_mode': LaunchConfiguration('control_mode'),
                         'vel_kp_pos': f('vel_kp_pos'),
                         'vel_kv': f('vel_kv'),
                         'vel_ki': f('vel_ki'),
                         'vel_k_att': f('vel_k_att'),
                         'vel_swing_k': f('vel_swing_k'),
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
                         'thrust_ratio': kt,
                         'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                         'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                         'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                         'takeoff_thrust_ratio': kt_to,
                         'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                         'kt_batt_v_full': f('kt_batt_v_full'),
                         'kt_batt_v_empty': f('kt_batt_v_empty'),
                         'kt_print_period_s': f('kt_print_period_s'),
                         'pose_timeout_s': f('pose_timeout_s'),
                         'safety_ref_timeout_s': f('safety_ref_timeout_s'),
                         **map_tracker}],
            output='screen'))

    # ── Central fleet manager (tethered fleet only) ─────────────────────────
    nodes.append(Node(
        package='fleet_manager', executable='fleet_manager', name='fleet_manager',
        # manage the newcomer too: it must disarm on LAND and on an abort like the others
        # (R0575/R0577: left armed on the floor after LAND; an abort never reached it).
        # It exists only with the approach chain: a managed drone with no tracker fails ARM.
        parameters=[{'num_drones': n + (1 if truthy(context, 'enable_approach') else 0)}],
        output='screen'))

    # ── Dissipative reference generator with 1 reserved ATTACH node ─────────
    nodes.append(Node(
        package='dissipative_planner', executable='dissipative_planner', name='dissipative_planner',
        parameters=[{'num_drones': n,
                     'reserved_attach': i_('reserved_attach'),
                     'attach_cable_len': float(LaunchConfiguration('attach_cable_len').perform(context).strip()
                                               or LaunchConfiguration('cable_len').perform(context)),
                     'attach_central': b('attach_central'),
                     'attach_handout': b('attach_handout'),
                     'diss_t_handout': f('attach_t_handout'),
                     'attach_ff_ramp_s': f('attach_ff_ramp_s'),
                     'attach_elev_deg': f('attach_elev_deg'),
                     'diss_balanced_tensions': b('diss_balanced_tensions'),
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
                     'z_ki_in_orbit': b('z_ki_in_orbit'),
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
                     'diss_handout_tension_blend': b('diss_handout_tension_blend'),
                     'diss_wrench_true_attitude': b('diss_wrench_true_attitude'),
                     'attach_traj_hold_s': f('attach_traj_hold_s'),
                     'attach_seek_m': f('attach_seek_m'),
                     'attach_t_start_new': f('attach_t_start_new'),
                     'attach_blend_balanced': b('attach_blend_balanced'),
                     'attach_datum_shift': b('attach_datum_shift'),
                     'attach_moving': b('attach_moving'),
                     'attach_approach_direct': b('attach_approach_direct'),
                     'attach_traj_hold_mode': LaunchConfiguration('attach_traj_hold_mode'),
                     'hold_resume_tilt_deg': f('hold_resume_tilt_deg'),
                     'hold_max_s': f('hold_max_s'),
                     'handover_blend_s': f('handover_blend_s'),
                     'diss_ki_load': f('diss_ki_load'),
                     'diss_a_i_load_max': f('diss_a_i_load_max'),
                     'diss_trim_share_weighted': b('diss_trim_share_weighted'),
                     'net_land_z': f('net_land_z'),
                     'attach_approach': (not sil) and (not enable_approach_mpc) and (not partner),
                     'approach_hold': not partner,
                     'partner_handoff_topic': '/join_planner/handoff_ready' if partner else '',
                     'start_attached': partner_attached,
                     'partner_release': partner_attached,
                     'attach_velocity_newcomer': (
                         LaunchConfiguration('attach_control_mode').perform(context).strip()
                         == 'velocity'),
                     # rig: tether latches ON at ARM, OFF at their detach
                     **({'detach_magnet': True} if real else {}),
                     **map_planner}] + planner_options(context),
        output='screen'))

    # ════════════════ APPROACH DRONE (x3_drone3) CHAIN ════════════════
    # Gate the whole approach chain with enable_approach:=false to bring up ONLY the 3
    # tethered drones (isolates the tethered lift from the approach-drone nodes when
    # debugging). Default true.
    enable_approach = (LaunchConfiguration('enable_approach').perform(context).lower()
                       in ('1', 'true', 'yes'))
    if real and enable_approach:
        # the newcomer is hardwired as drone ATTACH_DRONE_ID: any other n puts a carrier and
        # the newcomer's mux on the same radio
        if n != ATTACH_DRONE_ID:
            raise RuntimeError(f'the rig attach with the approach chain needs num_drones:='
                               f'{ATTACH_DRONE_ID} (the newcomer is drone {ATTACH_DRONE_ID}), '
                               f'got {n}; or enable_approach:=false')
        # the collaborator's approach MPC is unflown on hardware and unstable on the linear
        # plant (decisions 2026-09-25, R0522), and a /magnet/command ON arms it (ATTACH)
        if enable_approach_mpc:
            raise RuntimeError('the approach MPC is not for the rig; add '
                               'enable_approach_mpc:=false (our tracker flies the approach) '
                               'or partner:=true')
    # rig magnets (docstring, RIG): weld_radius 0 blocks the weld, so the newcomer's magnet
    # must never come on (a physical catch with no weld declared is R8b's abort)
    dry_approach = real and enable_approach and float(
        LaunchConfiguration('weld_radius').perform(context)) <= 0.0
    if dry_approach and partner_attached:
        raise RuntimeError('rig weld_radius:=0 blocks the weld, but partner_attached '
                           'starts drone 3 welded: its release needs the magnet manager')
    if real and truthy(context, 'real_io'):
        n_radio = n + (1 if enable_approach else 0)
        newcomer = LaunchConfiguration('attach_magnet_initial').perform(context).strip().upper()
        if newcomer == 'AUTO':
            newcomer = 'ON' if partner_attached else 'OFF'
        latches = rig_magnet_latches(
            context, [LaunchConfiguration('magnet_initial').perform(context)] * n
            + [newcomer] * (n_radio - n))
        if dry_approach and latches[ATTACH_DRONE_ID] != 'OFF':
            raise RuntimeError('rig weld_radius:=0 (no weld) with drone 3\'s magnet ON '
                               'would catch the ring with no weld declared; latch it OFF')
        nodes += rig_io(launch_dir, n_radio, latches,
                        attach=enable_approach)
    if not enable_approach:
        return nodes

    d = ATTACH_DRONE_ID
    dn = ATTACH_DRONE_NAME
    elrs = f'/drone_{d}/ELRSCommand'

    # sim interface for the STANDALONE drone (not covered by the rviz/num_drones launch).
    if sim_io:
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f'motor_bridge_{d}',
            arguments=[f'/{dn}/gazebo/command/motor_speed'
                       f'@actuator_msgs/msg/Actuators]ignition.msgs.Actuators']))
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f'imu_bridge_{d}',
            arguments=[f'/{dn}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'],
            remappings=[(f'/{dn}/imu', f'/drone_{d}/imu')]))
        nodes.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge', name=f'pose_bridge_{d}',
            arguments=[f'/model/{dn}/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V']))

        # mocap for the standalone drone -> /drone_3/motion_capture_state (state for both the
        # approach MPC and our tracker). pose_index picks the body link from the PoseArray.
        nodes.append(Node(
            package='simulation_communication', executable='payload_mocap_emulator',
            name=f'mocap_{d}',
            parameters=[{'drone_id': d, 'drone_name': dn,
                         'pose_topic': f'/model/{dn}/pose',
                         'pose_index': i_('attach_pose_index'),
                         'publish_payload': False,
                         # same cap as the carriers' emulators (rviz launch mocap_hz, 120):
                         # unthrottled, drone 3's 500 Hz state fed every consumer (profile
                         # 2026-09-26)
                         'max_publish_hz': f('mocap_hz')}]))

        # betaflight inner loop: /drone_3/ELRSCommand -> /x3_drone3/.../motor_speed. Its pose sub
        # is remapped from the nested default to the standalone pose topic.
        nodes.append(Node(
            package='simulation_communication', executable='payload_betaflight_comm',
            name=f'bf_comm_{d}',
            parameters=[{'drone_id': d, 'drone_name': dn, 'parent_model': PARENT_MODEL,
                         'rate_source': LaunchConfiguration('rate_source'),
                         'imu_topic': f'/drone_{d}/imu'}],
            remappings=[(f'/model/{PARENT_MODEL}/model/{dn}/pose', f'/model/{dn}/pose')]))

    if not sil:
        # ELRSCommand MUX: forwards APPROACH (_tejen) until the weld, then OURS (_diss).
        # Rig: no magnet merge; the forwarded channel 6 is left as sent and the radio's
        # latch (the magnet manager's) overwrites it
        rig_magnet = ({'magnet_command_topic': '',
                       'magnet_channel': i_('magnet_channel')} if real else {})
        nodes.append(Node(
            package='drone_magnet', executable='elrs_mux', name=f'elrs_mux_{d}',
            parameters=[{'drone_id': d, 'latch': True,
                         'approach_stream': enable_approach_mpc or partner,
                         'handoff_topic': '/join_planner/handoff_ready' if partner else '',
                         'release_topic': '/partner/release' if partner_attached else '',
                         **rig_magnet}],
            output='screen'))

    # (a) collaborator's approach MPC -> pre-mux _tejen. Follows /join_planner/reference.
    # Gated separately (enable_approach_mpc:=false) since it is the one heavy node (acados
    # codegen at startup) -- lets you run the rest of the approach chain without it.
    # In SIL the whole approach chain is replaced by the bench's stand-in controller +
    # deterministic weld (decision D5, docs/design/sil_bench.md §4).
    if enable_approach_mpc:
        nodes.append(Node(
            package='controller_mpc_payload', executable='main', name=f'approach_mpc_{d}',
            parameters=[{'drone_id': d,
                         'use_external_reference': True,
                         'external_reference_topic': '/join_planner/reference',
                         # kT for the SOLO approach: the secant gain of an unloaded
                         # hover (no load share, no cable pull), derived like the
                         # fleet's; an explicit thrust_ratio is used verbatim.
                         'mpc_thrust_ratio': kt_solo,
                         'enable_thrust_ratio_ukf': b('approach_kt_ukf'),
                         'enable_thrust_ratio_feedback': b('approach_kt_feedback'),
                         'thrust_ratio_estimator_backend': 'full_model_kt_ukf',
                         # one command path (Wesley 2026-09-29): the manager's accepted
                         # TAKEOFF on /drone_3/command arms and starts it; never the broadcast
                         'takeoff_implies_arm': True}],
            # ELRSCommand out -> pre-mux _tejen. Commands IN <- /drone_3/command, where the fleet
            # manager publishes TAKEOFF once it has accepted it (a refused TAKEOFF never reaches
            # it); emergencies reach the drone through the mux latch.
            remappings=[(elrs, f'{elrs}_tejen'),
                        ('drone_command', f'/drone_{d}/command')], output='screen'))

    # (b) our dissipative tracker for drone 3 -> pre-mux _diss. The fleet manager arms it and
    # sends it TAKEOFF like the carriers (one command path, Wesley 2026-09-29: a broadcast
    # remap let it take off when the manager refused), so it is already flying-ready when the
    # weld hands it authority. Pre-weld it has no reference (dissipative streams drone 3's ref only
    # after attach) so it just publishes an armed-idle command on _diss, which the mux does NOT
    # forward -- the real drone stays on the approach MPC until the weld. takeoff_spool_s=0 so the
    # mid-air takeover applies full MPC hover thrust immediately instead of ramping up from a floor
    # (a spool ramp would drop the drone at the handoff).
    # NOTE: deliberately NO adaptive-kT wiring on the newcomer's tracker, even when
    # the tethered drones have it on. Drone 3's thrust regime CHANGES at the weld --
    # free-flight kT is ~29.5, loaded-hover kT is ~33 -- so a learn-then-lock during
    # its approach would lock the pre-weld value and then fly the post-weld phase on
    # it. It stays on the scheduled/fixed kT, which tracks the operating point through
    # the transition by construction.
    nodes.append(Node(
        package='tracker', executable='tracker', name=f'tracker_{d}',
        parameters=[{'drone_id': d,
                     # the newcomer's tracker follows the SAME control architecture as the
                     # tethered ones. It did not (2026-09-09): control_mode never reached
                     # this node, so every "velocity" attach arm flew the newcomer on the
                     # no-integrator MPC and it hovered 13 cm under its reference (R0185).
                     'control_mode': (LaunchConfiguration('attach_control_mode').perform(context).strip()
                                      or LaunchConfiguration('control_mode').perform(context)),
                     'vel_kp_pos': f('vel_kp_pos'),
                     'vel_kv': f('vel_kv'),
                     'vel_ki': f('vel_ki'),
                     'vel_k_att': f('vel_k_att'),
                         'vel_swing_k': f('vel_swing_k'),
                     'cable_ff_scale': f('cable_ff_scale'),
                     'attitude_ff': b('attitude_ff'),
                     'cable_source': LaunchConfiguration('cable_source'),
                     'w_thr_ref': f('w_thr_ref'),
                     'x0_relax_frac': f('x0_relax_frac'),
                     'ref_time_shift': b('ref_time_shift'),
                         'ground_idle': b('ground_idle'),
                     'cable_accel_cap': f('cable_accel_cap'),
                     'payload_rest_z': f('payload_rest_z'),
                     'takeoff_spool_s': 0.0,
                     'thrust_ratio': kt,
                     'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                     'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                     'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                     'takeoff_thrust_ratio': kt_to,
                     'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                     'kt_batt_v_full': f('kt_batt_v_full'),
                     'kt_batt_v_empty': f('kt_batt_v_empty'),
                     # one-line ~2 Hz health log for the newcomer, to trace why it sinks/falls
                     # after the weld (z vs ref, xy error, throttle saturation, cable FF).
                     'pose_timeout_s': f('pose_timeout_s'),
                         'safety_ref_timeout_s': f('safety_ref_timeout_s'),
                     # a free drone 1.2 m from the origin: the default node-0 box
                     # (x*(1-eps), x*(1+eps)) inverts on its negative y, R0523/R0526
                     'x0_relax_symmetric': b('x0_relax_symmetric'),
                     'enable_diag_log': True,
                     **map_tracker}],
        # In SIL there is no mux, so this tracker publishes straight onto
        # /drone_3/ELRSCommand, which the bench plant consumes. Everything else about
        # the node is unchanged, so it is the same tracker the Gazebo run flies.
        remappings=([] if sil else [(elrs, f'{elrs}_diss')]), output='screen'))

    # SIL stops here: the bench replaces the magnet/approach chain below with a
    # scripted weld and a /magnet/object_attached publish (decision D5).
    if sil:
        return nodes

    # attach target: republish the shared payload's mocap as the join planner's magnet-tip
    # target (PoseStamped + TwistStamped). Without this the planner never publishes a
    # reference and the approach MPC never arms. Partner mode needs it too: our tracker
    # flies the last descent after the handoff.
    if True:
      nodes.append(Node(
        package='drone_magnet', executable='attach_target_publisher', name='attach_target_publisher',
        parameters=[{'payload_state_topic': '/payload/motion_capture_state',
                     'pose_topic': '/attach_target/pose',
                     'twist_topic': '/attach_target/twist',
                     # OFF-CENTRE weld point (world frame): with diss_balanced_tensions the 4th
                     # welds at a ring point (moment arm != 0) so the fleet reconfigures level.
                     # 0,0 = centre weld (central lifter). Keep within the manager weld_radius.
                     'x_offset': f('attach_x_offset'),
                     'y_offset': f('attach_y_offset'),
                     'z_offset': 0.05}],
        output='screen'))

    # approach planner: emits /join_planner/reference toward the payload attach target with
    # obstacle avoidance; watches /magnet/object_attached to stop once welded.
    if not partner:
      nodes.append(Node(
        package='drone_magnet', executable='online_join_planner', name='online_join_planner',
        parameters=[{'mission_mode': 'join',
                     'drone_state_topic': f'/drone_{d}/motion_capture_state',
                     'attachment_pose_topic': '/attach_target/pose',
                     'attachment_twist_topic': '/attach_target/twist',
                     'reference_topic': '/join_planner/reference',
                     'object_attached_topic': '/magnet/object_attached',
                     'magnet_command_topic': '/magnet/command',
                     # Obstacle avoidance: treat the REAL tethered fleet drones as the moving
                     # obstacles the approach must route around (was a fake-world stand-in that
                     # nothing publishes here, so avoidance was inert). Same MotionCaptureState
                     # type/topics the planner already tracks -- pose.position = centre,
                     # twist.linear = velocity. The approach drone (id d) avoids drones 0..n-1.
                     # NOTE the geometric tension: the payload sits INSIDE the ring of tethered
                     # drones (each ~cable_len*cos(elev) ~= 0.38 m out), so a large safety sphere
                     # seals the corridor to the payload and the approach oscillates HOLD_FOR_
                     # OBSTACLE forever. obstacle_safety_radius must be small enough to leave a gap
                     # (default 0.15 here, vs the node's 0.35), or set enable_obstacle_avoidance:=
                     # false to weld without fleet avoidance (they are teammates, not obstacles).
                     'enable_obstacle_avoidance': b('enable_obstacle_avoidance'),
                     'enable_obstacle_diagnostics': b('enable_obstacle_avoidance'),
                     'obstacle_safety_radius': f('obstacle_safety_radius'),
                     'obstacle_state_topics': [f'/drone_{i}/motion_capture_state'
                                               for i in range(n)],
                     # Once ATTACH has started the approach and the drone matches position/velocity
                     # over the payload, actually LOWER the magnet to the payload (MATCH_VELOCITY ->
                     # DESCEND_TO_ATTACHMENT is gated on this). Without it the drone hovers above and
                     # never welds. Safe here because the whole approach is ATTACH-gated already.
                     'auto_descend': True}],
        output='screen'))

    # RIG: the weld is declared from mocap alone (the tip body on /magnet_tip_pose, the ring
    # body on /payload/motion_capture_state); no joint to command, the magnet is drone 3's
    # radio latch, which this node alone drives (none on a dry approach).
    if real:
        nodes.append(Node(
            package='drone_magnet', executable='magnet_attachment_manager',
            name='magnet_attachment_manager',
            parameters=[{'magnet_tip_pose_topic': '/magnet_tip_pose',
                         'magnet_tip_pose_msg_type': 'pose_stamped',
                         'attached_at_start': partner_attached,
                         'object_pose_topic': '/payload/motion_capture_state',
                         'object_pose_msg_type': 'mocap_state',
                         'object_x_offset': f('attach_x_offset'),
                         'object_y_offset': f('attach_y_offset'),
                         'use_fallback_object_pose': False,
                         'attach_radius': f('weld_radius'),
                         'attach_speed_threshold': 0.05,
                         'attach_dwell_time_s': 0.15,
                         'rel_vel_filter_tau_s': f('weld_vel_filter_s'),
                         'velocity_clock': LaunchConfiguration('weld_velocity_clock'),
                         'object_attached_topic': '/magnet/object_attached',
                         'command_backend': 'none',
                         'detach_on_start': False,
                         'enable_elrs_magnet_output': False,
                         'magnet_latch_topic': '' if dry_approach else f'/drone_{d}/magnet'}],
            output='screen'))
        return nodes

    # magnet tip pose (world) from the drone's poses -> /magnet_tip_pose. Verified array order:
    # the magnet_tip_link (model-relative) is index 0 and the body WORLD pose is index -1, so we
    # compose the relative tip against the body world pose. (Matches the node's own defaults.)
    nodes.append(Node(
        package='drone_magnet', executable='magnet_tip_publisher', name='magnet_tip_publisher',
        parameters=[{'x3_pose_topic': f'/model/{dn}/pose',
                     'magnet_tip_topic': '/magnet_tip_pose',
                     'magnet_index': 0,
                     'drone_index': -1,
                     'x3_link_poses_are_relative': True}]))

    # ROS->gz bridges for the magnet DetachableJoint, mirroring the per-drone detach_bridge_{i}
    # above (the PROVEN detach path). The magnet manager publishes std_msgs/Empty on these ROS
    # topics and the bridge forwards to the gz DetachableJoint. This replaces the fragile
    # `gz topic` subprocess backend, which was silently failing to reach the joint (the weld
    # persisted -> pendulum pulled sideways, armed drone dragged the payload).
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='payload_attach_bridge',
        arguments=['/payload/attach@std_msgs/msg/Empty]gz.msgs.Empty']))
    nodes.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='payload_detach_bridge',
        arguments=['/payload/detach@std_msgs/msg/Empty]gz.msgs.Empty']))

    # magnet manager: auto-welds (ROS /payload/attach -> bridge -> gz) when the tip is near &
    # slow, and announces /magnet/object_attached (Bool) -- the single signal that flips the mux
    # AND folds drone 3 into the dissipative network. Uses the ros_topic backend so attach/detach
    # go through the bridge above (same mechanism as the working tethered-drone detach).
    nodes.append(Node(
        package='drone_magnet', executable='magnet_attachment_manager', name='magnet_attachment_manager',
        parameters=[{'magnet_tip_pose_topic': '/magnet_tip_pose',
                     'attached_at_start': partner_attached,
                     'object_pose_topic': f'/model/{PARENT_MODEL}/model/payload/pose',
                     # payload PoseArray: index 1 is the body's WORLD pose (same entry the mocap
                     # uses), index 0 is the frozen/relative model-frame pose. Reading 0 put the
                     # payload at ~ground so distance was ~0.6 m and it never welded. Also drop the
                     # ground fallback (z=0.08): with the load LIFTED, a fallback would weld at the
                     # wrong place -- better to wait for the real pose.
                     'object_pose_index': 1,
                     # MUST match attach_target_publisher's offset: the weld triggers on tip
                     # distance to (payload centre + this offset), so the tip welds at the
                     # off-centre ring point instead of the centre (enables reconfiguration).
                     'object_x_offset': f('attach_x_offset'),
                     'object_y_offset': f('attach_y_offset'),
                     'use_fallback_object_pose': False,
                     # tip-to-reference distance at weld ~0.10 m vertical (the reference is
                     # offset off-centre by object_x/y_offset, so the horizontal term drops
                     # out). 0.15 gives margin without welding mid-descent. Launch arg.
                     'attach_radius': f('weld_radius'),   # magnet manager's threshold
                     # partner rejoin: weld only once the tip has stopped (the approach
                     # ends in a hold 4 cm above the plate). Welding mid-descent let the
                     # still-descending drone push the now-rigid rod into the ring: carriers
                     # dropped 0.25 m and the ring tipped 25-35 deg (R0640, R0642)
                     **({'attach_speed_threshold': 0.05} if partner else {}),
                     'rel_vel_filter_tau_s': f('weld_vel_filter_s'),
                     'velocity_clock': LaunchConfiguration('weld_velocity_clock'),
                     'object_attached_topic': '/magnet/object_attached',
                     'command_backend': 'ros_topic',
                     'ros_attach_topic': '/payload/attach',
                     'ros_detach_topic': '/payload/detach'}],
        output='screen'))

    return nodes
