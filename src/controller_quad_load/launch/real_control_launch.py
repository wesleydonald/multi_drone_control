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
  * thrust_ratio defaults to 35.2 here -- the gain ABOVE the 0.185 throttle offset of
    the affine map measured on 30 Sep (thrust_offset, thrust_offset_v_slope). kt_trim
    is off by default (kt_trim:=true measures the rest within +-25 % in steady hover;
    it cannot learn above the tracker's 0.8 throttle cap). kt_batt_sag_frac:=0.10 turns
    on a linear derate with pack voltage once you have measured the sag.
  * cable_len / load_mass MUST match your physical rig (not the sim SDF).
    load_mass defaults to 0.86 here (the ring). Changing it recompiles the acados .so on the
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
        DeclareLaunchArgument('num_drones', default_value='4'),
        # ARM waits for each flight controller's own armed report (CRSF flight mode)
        DeclareLaunchArgument('require_fc_armed', default_value='true'),
        # MUST match the physical cables.
        DeclareLaunchArgument('cable_len', default_value='0.55'),
        # where the rim attachments are (deg, load frame, sized by num_drones); '' = even
        # ring. Clock face: 3 o'clock = 0, 12 = 90, 9 = 180, 6 = 270.
        DeclareLaunchArgument('attach_azimuths_deg', default_value=''),
        # Where the cables attach on the payload, LOAD frame: a ring of radius
        # attach_radius at height attach_z above the CoM, azimuth 2*pi*i/n. On
        # hardware there is no SDF, so these two numbers are the planner's ONLY
        # description of the payload -- measure them. LOAD_INERTIA in
        # planner_node.py does not scale with attach_radius either.
        DeclareLaunchArgument('attach_radius', default_value='0.225'),
        DeclareLaunchArgument('attach_z', default_value='0.0'),
        # Physical payload mass (kg). LOAD_INERTIA in planner_node.py is a
        # hardcoded constant and does NOT scale with this.
        DeclareLaunchArgument('load_mass', default_value='0.86'),
        DeclareLaunchArgument('drone_mass', default_value='0.55'),   # WEIGH the airframe with its pack; 0.64 is the sim model
        DeclareLaunchArgument('start_taut', default_value='false'),
        DeclareLaunchArgument('handover_elev_deg', default_value='45.0'),
        # Frozen hold after handover, before the lift ramp starts. Shorter = faster
        # takeoff; too short and the fleet starts climbing before it has settled on
        # the latched config.
        DeclareLaunchArgument('handover_settle_s', default_value='1.0'),
        DeclareLaunchArgument('creep_vel', default_value='0.2'),   # m/s creep sweep rate before the handover
        # floor start: every rod's pull ramps in together over this long before the lift (0 = off)
        DeclareLaunchArgument('pretension_s', default_value='3.0'),
        DeclareLaunchArgument('ff_cap_release_s', default_value='2.0'),   # breakaway cap back to 1 after lift start; 0 = held all flight (pre 2 Oct)
        # trust band for the rod lengths measured at the hand-over (typed cable_len +-tol, spread)
        DeclareLaunchArgument('rod_tol_frac', default_value='0.25'),
        DeclareLaunchArgument('rod_spread_m', default_value='0.08'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        # Climb rate (m/s) -- the dominant term in takeoff duration. The ramp is
        # eased at both ends (LIFT_SOFT_* in planner_node.py), so peak accel stays
        # well under the raw rate. HOLD test: lift_ramp_vel:=0.0 (no lift, just
        # hold the taut config).
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.1'),
        DeclareLaunchArgument('z_ki', default_value='0.4'),      # planner height integral, 0 = off (card 2026-09-24_planner_offset)
        # a small safety net only (Wesley 2026-09-30): the model is judged with z_ki 0
        DeclareLaunchArgument('z_i_max', default_value='0.15'),
        DeclareLaunchArgument('z_i_gate', default_value='0.25'),  # integral runs only while |miss| < this
        # rig fallback z_taut_gate:=0.6 only if the planner pivot model is off
        DeclareLaunchArgument('z_taut_gate', default_value='0.9'),
        # rod pivot below the drone centre (m, body z): the rig joint sits 4 cm under it
        DeclareLaunchArgument('pivot_offset_z', default_value='-0.04'),
        # wall cap on a planner solve (s) so a hard solve cannot stall the 10 Hz loop; 0 = off (sim)
        DeclareLaunchArgument('solve_budget_s', default_value='0.06'),
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        # Cable compensation. ON is the correct flight config. Zero only for a
        # deliberate A/B (cable_ff_scale:=0.0 cable-blind; attitude_ff:=false level).
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
        # 35.2 = the gain above the 0.185 offset of the affine map below (was 24 on the
        # old linear map). The sim launches use their own secant gain: Gazebo's motor
        # model is quadratic, so the two genuinely differ.
        DeclareLaunchArgument('thrust_ratio', default_value='35.2'),
        # per-drone thrust-gain trim (kt_trim.py, card 2026-09-23_kt_trim.md): off until the matrix passes
        DeclareLaunchArgument('kt_trim', default_value='false'),
        DeclareLaunchArgument('kt_trim_max', default_value='0.25'),
        DeclareLaunchArgument('kt_trim_tau', default_value='1.5'),
        # tracker throttle ceiling; 0.8 unless typed (supervisor allows up to 1.0, 2026-09-30)
        DeclareLaunchArgument('throttle_max', default_value='0.8'),
        # affine thrust map, identified 30 Sep 2026 (RIG-0930-ladder4): throttle = offset
        # + 0.507*mass - 0.022*(V - 23.5). thrust_ratio is then the gain ABOVE the offset,
        # g/(0.507*0.55 kg) = 35.2. thrust_offset:=0 thrust_ratio:=21.7 is the old map.
        DeclareLaunchArgument('thrust_offset', default_value='0.185'),
        DeclareLaunchArgument('thrust_offset_v_slope', default_value='0.022'),
        DeclareLaunchArgument('thrust_v_ref', default_value='23.5'),
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
        # ON: match each drone to the nearest nominal ring slot at the first solve,
        # so you can place the drones ~cable_len out in ANY order (no need to line
        # drone 0 up with +x). Relabels I/O only -- no OCP recompile.
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        # measure each rod from mocap at handover instead of trusting cable_len (2026-09-23)
        DeclareLaunchArgument('measure_rod_len', default_value='true'),
        # LOAD reference after the lift tops out: 'hover', 'line_x', 'circle',
        # 'fig_8', 'spin' (circle + one full load yaw). circle/fig_8/spin use
        # traj_radius; line_x uses traj_distance; all use traj_speed. Prove hover
        # first, then keep traj_speed slow.
        # Experiment T1 (finding F10): terminal cost tracks ref_vel instead
        # of commanding a stop at the end of the horizon. Default false =
        # historical behaviour, so this only changes a run you asked it to.
        DeclareLaunchArgument('terminal_vel_ref', default_value='false'),
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
                         'terminal_vel_ref': b('terminal_vel_ref'),
                         'cable_ff_scale': f('cable_ff_scale'),
                         'attitude_ff': b('attitude_ff'),
                         'cable_source': LaunchConfiguration('cable_source'),
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
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n,
                     'require_fc_armed': LaunchConfiguration('require_fc_armed').perform(context).lower() == 'true'}],
        output='screen'))

    # ── Centralized cable-suspended load planner ──────────────────────────────
    nodes.append(Node(
        package='controller_load_mpc', executable='planner', name='load_planner',
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
                     'traj_radius': f('traj_radius')}],
        output='screen'))

    return nodes


# Rig defaults = the 30 Sep 2026 lab values (Wesley's word): four drones on the even ring,
# measured drone 0.55 kg, rod 0.55 m (pivot 4 cm below the drone centre: pivot_offset_z),
# magnets at r 0.225 m, affine thrust (kT 35.2 above the 0.185 offset, kt_trim off), cap 0.8,
# pretension, widened rod trust band and height integral.
def generate_launch_description():
    return LaunchDescription(_args() + [OpaqueFunction(function=launch_setup)])
