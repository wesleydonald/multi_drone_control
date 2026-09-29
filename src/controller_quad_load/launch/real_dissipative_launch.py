"""
real_dissipative_launch.py  —  TERMINAL 2 (control, DISSIPATIVE variant)
------------------------------------------------------------------------
For M2 (partner_m2) the rig flies dissipative_launch.py real:=true instead (Wesley
2026-09-28); this file stays the two-terminal carry/detach launch of the R1-R7 sheets.

Real-world control layer using the DECENTRALIZED DISSIPATIVE reference generator
instead of the centralized OCP planner. This is the hardware twin of
dissipative_launch.py, exactly as real_control_launch.py is the hardware twin of
the control half of mpc_quad_load_launch.py.

Same wiring as real_control_launch.py -- per-drone cable-aware MPC trackers plus
the central fleet manager -- but load_planner is replaced by
controller_dissipative's `dissipative` node. Every Gazebo bridge from
dissipative_launch.py (motor / imu / detach / betaflight_comm) is dropped: real
radios and real MoCap replace them, and there is no /clock on hardware
(use_sim_time:=false).

TAKEOFF IS UNCHANGED. The dissipative node is an OCP-takeoff subclass of
LoadPlanner: it flies the identical centralized OCP until the first
/fleet/detach, then hands over to the spring-damper network. With no detach ever
published it behaves like real_control_launch.py throughout -- which is exactly
what you want for a first hardware bring-up of this stack.

DETACH on the rig: /fleet/detach <k> (RViz button with real_io_launch detach:=true,
or a topic pub). reconfig_mode:=ocp resizes the OCP in place (the verified-best sim
path); detach_magnet:=true releases /drone_<k>/magnet on the same tick. With
neither published nothing changes: reserved_attach defaults to 0, so no attach
topics are subscribed either.

Start this AFTER real_io_launch.py (terminal 1) is up and MoCap is streaming --
the trackers and the dissipative node block waiting for the first
/drone_<id>/motion_capture_state and /payload/motion_capture_state poses.

Run (terminal 2, after terminal 1). Pass the SAME num_drones to both terminals:
    ros2 launch controller_quad_load real_io_launch.py num_drones:=3 \
        drone0_serial:=/dev/QUAD1 drone1_serial:=/dev/QUAD2 drone2_serial:=/dev/QUAD3
    ros2 launch controller_quad_load real_dissipative_launch.py num_drones:=3
then drive the fleet:
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: LAND}"

HARDWARE NOTES (same as real_control_launch.py, plus the network tuning):
  * thrust_ratio 24 -- the typed prop/motor kT at full battery health (the 09-16 free
    hover measured ~22). kt_trim (default on) measures the rest within +-25 % once the
    drone is in steady hover; it cannot learn above the tracker's 0.6 throttle cap.
  * cable_len / load_mass MUST match your physical rig.
  * The diss_* defaults are the tuned RIGID-SHORT values for cable_len=0.5,
    load_mass=0.4, drone_mass=0.6 (see dissipative_network.py). load_mass
    defaults to 0.86 here, so the stiffnesses are NOT tuned for this payload --
    they only matter after a network detach, but retune before relying on them.
  * reconfig_mode defaults to network and detach_magnet to false here: a rig DETACH
    types reconfig_mode:=ocp detach_magnet:=true (the network detach tilted the ring
    26-48 deg in sim).
  * diss_elev_deg is the network's nominal cable elevation and is independent of
    handover_elev_deg (45.0 here: the creep floor start).
  * Prove a taut hover (load_traj:=hover) before any circle/fig_8/spin.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def _args():
    return [
        # Must match the num_drones given to real_io_launch.py.
        DeclareLaunchArgument('num_drones', default_value='2'),
        # MUST match the physical rig, not the sim SDF.
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        # where the rim attachments are (deg, load frame, sized by num_drones); '' = even
        # ring. Clock face: 3 o'clock = 0, 12 = 90, 9 = 180, 6 = 270.
        DeclareLaunchArgument('attach_azimuths_deg', default_value=''),
        DeclareLaunchArgument('load_mass', default_value='0.86'),
        DeclareLaunchArgument('drone_mass', default_value='0.64'),   # WEIGH the airframe with its pack; 0.64 is the sim model
        # Real rig starts taut and level: no creep phase, no handover arc.
        DeclareLaunchArgument('start_taut', default_value='false'),
        DeclareLaunchArgument('handover_elev_deg', default_value='45.0'),
        DeclareLaunchArgument('handover_settle_s', default_value='1.0'),
        DeclareLaunchArgument('creep_vel', default_value='0.2'),   # m/s creep sweep rate before the handover
        DeclareLaunchArgument('target_z', default_value='0.6'),
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.20'),
        DeclareLaunchArgument('z_ki', default_value='0.4'),      # planner height integral, 0 = off (card 2026-09-24_planner_offset)
        DeclareLaunchArgument('z_i_max', default_value='0.15'),
        DeclareLaunchArgument('z_taut_gate', default_value='0.9'),
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        DeclareLaunchArgument('cable_ff_scale', default_value='1.0'),
        DeclareLaunchArgument('attitude_ff', default_value='true'),
        DeclareLaunchArgument('cable_source', default_value='model'),
        DeclareLaunchArgument('payload_rest_z', default_value='0.05'),
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.5'),
        # ── kT (thrust ratio) ───────────────────────────────────────────────
        # The ONE number the tracker's thrust model uses: it assumes a = kT*throttle
        # and NOTHING estimates or reschedules it in flight. The adaptive UKF and the
        # thrust_quad_c airborne schedule were both removed on 2026-08-05.
        #
        # 24.0 = the MEASURED kT of these airframes on a pack at full health. (The sim
        # launches use ~31 instead: Gazebo's motor model is quadratic, so a linear
        # model there has to use the secant gain at hover. The two genuinely differ.)
        DeclareLaunchArgument('thrust_ratio', default_value='24.0'),
        # per-drone thrust-gain trim (kt_trim.py, card 2026-09-23_kt_trim.md): off until the matrix passes
        DeclareLaunchArgument('kt_trim', default_value='true'),
        DeclareLaunchArgument('kt_trim_max', default_value='0.25'),
        DeclareLaunchArgument('kt_trim_tau', default_value='1.5'),
        # kT used before the drone is airborne. 0 = same as thrust_ratio, which is
        # right for a GROUND takeoff: the deliberate takeoff under-assumption exists
        # only to pop drones off the stands of a taut sim air-start. Set it below
        # thrust_ratio only if this rig genuinely needs extra oomph to break ground.
        DeclareLaunchArgument('takeoff_thrust_ratio', default_value='0.0'),
        # BATTERY DERATE. kT falls as the pack sags:
        #     kT = thrust_ratio * (1 - kt_batt_sag_frac * depletion)
        #     depletion = clip((v_full - v)/(v_full - v_empty), 0, 1)
        # OFF (0.0) by default -- 24.0 above is the full-health number, and the sag
        # fraction has NOT been measured on this rig yet. To calibrate: hover the same
        # load on a full pack and on a nearly-flat one and compare hover throttle.
        # Voltage comes from /drone_N/telemetry (ELRS). With no telemetry the derate
        # is skipped and kT stays at thrust_ratio.
        DeclareLaunchArgument('kt_batt_sag_frac', default_value='0.0'),
        DeclareLaunchArgument('kt_batt_v_full', default_value='25.2'),   # 6S 4.20 V/cell
        DeclareLaunchArgument('kt_batt_v_empty', default_value='21.0'),  # 6S 3.50 V/cell
        # Seconds between per-drone kT reports; 0 = silent.
        DeclareLaunchArgument('kt_print_period_s', default_value='1.0'),
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        # measure each rod from mocap at handover instead of trusting cable_len (2026-09-23)
        DeclareLaunchArgument('measure_rod_len', default_value='true'),
        # Experiment T1 (finding F10): terminal cost tracks ref_vel instead
        # of commanding a stop at the end of the horizon. Default false =
        # historical behaviour, so this only changes a run you asked it to.
        DeclareLaunchArgument('terminal_vel_ref', default_value='false'),
        # Tracker architecture (docs/design/velocity_loop.md): 'mpc' | 'velocity' |
        # 'velocity_after_handover'. Default 'mpc' = the historical hardware path; the
        # sim's Part A winner is velocity_after_handover + vel_ki 0 + diss_ki_load 1.0.
        # Gains are the sim values, unmeasured on the airframe. INDI stays sim-only.
        DeclareLaunchArgument('control_mode', default_value='mpc'),
        DeclareLaunchArgument('vel_kp_pos', default_value='2.0'),
        DeclareLaunchArgument('vel_kv', default_value='4.0'),
        DeclareLaunchArgument('vel_ki', default_value='1.0'),
        DeclareLaunchArgument('vel_k_att', default_value='8.0'),
        DeclareLaunchArgument('load_traj', default_value='hover'),
        DeclareLaunchArgument('traj_speed', default_value='0.4'),
        DeclareLaunchArgument('traj_distance', default_value='1.0'),
        DeclareLaunchArgument('traj_radius', default_value='0.5'),
        # Dissipative network tuning (DissipativeParams). These only take effect
        # after a detach; with no detach the OCP takeoff path owns the refs.
        # Reference-shaping for the NETWORK phase (see dissipative_node).
        #   net_horizon_preview: carry the load trajectory's future displacement along
        #     the published horizon instead of repeating node 0. ON by default -- logged
        #     A/B: payload lag 3.11 s -> 1.11 s, mean error 0.172 -> 0.081 m.
        #   net_traj_lean: tilt the reference cone onto g_eff = g - a_traj so the
        #     formation leads the load into the maneuver (the OCP's flatness relation).
        #     OFF by default: unvalidated, mini_plant cannot reproduce the radius
        #     deficit it targets. Fly it as an A/B against the logs.
        # Hand the fleet to the network once the OCP lift tops out (the sim's
        # dissipative_only behaviour). Default false = network only on a detach.
        DeclareLaunchArgument('auto_network_handover', default_value='false'),
        DeclareLaunchArgument('auto_handover_settle_s', default_value='1.5'),
        # Common-mode load trim (z-only integrator in the network); 0.0 = off.
        DeclareLaunchArgument('diss_ki_load', default_value='0.0'),
        DeclareLaunchArgument('diss_a_i_load_max', default_value='2.0'),
        DeclareLaunchArgument('diss_trim_share_weighted', default_value='false'),
        DeclareLaunchArgument('net_horizon_preview', default_value='true'),
        DeclareLaunchArgument('net_traj_lean', default_value='false'),
        DeclareLaunchArgument('diss_k_pay', default_value='40.0'),
        DeclareLaunchArgument('diss_k_anchor', default_value='40.0'),
        DeclareLaunchArgument('diss_c', default_value='6.0'),
        DeclareLaunchArgument('diss_k_ring', default_value='20.0'),
        DeclareLaunchArgument('diss_k_slot', default_value='18.0'),
        DeclareLaunchArgument('diss_node_mass', default_value='0.5'),
        DeclareLaunchArgument('diss_substeps', default_value='10'),
        DeclareLaunchArgument('diss_elev_deg', default_value='45.0'),
        # Floor the network descends the held load to on a network-phase LAND.
        DeclareLaunchArgument('net_land_z', default_value='0.06'),
        # ── How a fleet-size change is handled (THESIS_PLAN §12.2) ──────────
        # 'network' (default, verified): the dissipative network takes the fleet on a
        #     detach, redistributes, and carries the trajectory on itself.
        # 'ocp': the OCP is RESIZED in place to the new fleet size and keeps flying
        #     (R0111-R0114: ~1 deg settled tilt vs 26-48 deg on the network). Builds
        #     the n-1 solver at start: run tools/prebuild_planner.py first.
        DeclareLaunchArgument('reconfig_mode', default_value='network'),
        DeclareLaunchArgument('reconfig_hold_s', default_value='1.5'),
        # On the rig the physical release is the tether magnet (aux channel via
        # elrs_interface). true: the node publishes /drone_<k>/magnet OFF at the
        # detach so command and release are simultaneous (needs real_io_launch
        # magnet_initial:=ON). false (default): a helper toggles the magnet by hand.
        DeclareLaunchArgument('detach_magnet', default_value='false'),
        # fewest drones that may stay on the load after a detach (3 -> 2 capsizes in SIL)
        DeclareLaunchArgument('min_survivors', default_value='3'),
    ]


def launch_setup(context, *args, **kwargs):
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
            package='controller_quad_load', executable='controller',
            name=f'controller_{i}',
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
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'thrust_ratio': f('thrust_ratio'),
                         'kt_trim': ParameterValue(LaunchConfiguration('kt_trim'), value_type=bool),
                         'kt_trim_max': ParameterValue(LaunchConfiguration('kt_trim_max'), value_type=float),
                         'kt_trim_tau': ParameterValue(LaunchConfiguration('kt_trim_tau'), value_type=float),
                         'takeoff_thrust_ratio': f('takeoff_thrust_ratio'),
                         'kt_batt_sag_frac': f('kt_batt_sag_frac'),
                         'kt_batt_v_full': f('kt_batt_v_full'),
                         'kt_batt_v_empty': f('kt_batt_v_empty'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            output='screen'))

    # ── Central fleet manager ─────────────────────────────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n}], output='screen'))

    # ── Decentralized dissipative reference generator ─────────────────────────
    # Replaces controller_load_mpc/planner. OCP takeoff until /fleet/detach, then
    # the spring-damper network. reserved_attach is left at its default 0, so no
    # attach topics are created.
    nodes.append(Node(
        package='controller_dissipative', executable='dissipative',
        name='dissipative_controller',
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
                     'net_land_z': f('net_land_z')}],
        output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])
