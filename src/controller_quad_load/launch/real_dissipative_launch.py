"""
real_dissipative_launch.py  —  TERMINAL 2 (control, DISSIPATIVE variant)
------------------------------------------------------------------------
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

NO DETACH: simply never publish /fleet/detach. reserved_attach defaults to 0, so
no attach topics are subscribed either, and real_io_launch.py already hides the
RViz DETACH/ATTACH buttons. Nothing further is needed to disable them.

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
  * thrust_ratio 24 -- the measured prop/motor kT at full battery health, NOT
    the sim's ~31. FIXED: nothing estimates or reschedules it in flight.
  * cable_len / load_mass MUST match your physical rig.
  * The diss_* defaults are the tuned RIGID-SHORT values for cable_len=0.5,
    load_mass=0.4, drone_mass=0.6 (see dissipative_network.py). load_mass
    defaults to 0.1 here, so the stiffnesses are NOT tuned for this payload --
    they only matter after a detach, but retune before relying on them.
  * diss_elev_deg is the network's nominal cable elevation and is independent of
    handover_elev_deg (which is 0.0 on the taut real start).
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
        DeclareLaunchArgument('load_mass', default_value='0.1'),
        # Real rig starts taut and level: no creep phase, no handover arc.
        DeclareLaunchArgument('start_taut', default_value='true'),
        DeclareLaunchArgument('handover_elev_deg', default_value='0.0'),
        DeclareLaunchArgument('handover_settle_s', default_value='0.5'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.20'),
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
        DeclareLaunchArgument('kt_batt_v_full', default_value='16.8'),   # 4S 4.20 V/cell
        DeclareLaunchArgument('kt_batt_v_empty', default_value='14.0'),  # 4S 3.50 V/cell
        # Seconds between per-drone kT reports; 0 = silent.
        DeclareLaunchArgument('kt_print_period_s', default_value='1.0'),
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        # Experiment T1 (finding F10): terminal cost tracks ref_vel instead
        # of commanding a stop at the end of the horizon. Default false =
        # historical behaviour, so this only changes a run you asked it to.
        DeclareLaunchArgument('terminal_vel_ref', default_value='false'),
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
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
                         'payload_rest_z': f('payload_rest_z'),
                         'takeoff_spool_s': f('takeoff_spool_s'),
                         'thrust_ratio': f('thrust_ratio'),
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
                     'net_land_z': f('net_land_z')}],
        output='screen'))

    return nodes


def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])
