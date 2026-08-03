"""
mpc_quad_load_launch.py
-----------------------
PLANNER-DRIVEN, CABLE-AWARE stack for the cable-suspended payload, for ANY
fleet size. The centralized planner (controller_load_mpc) generates each drone's
reference trajectory AND its per-node cable tension acceleration (t*s/m), and the
per-drone cable-aware MPC (controller_quad_load) tracks it with the cable force in
its prediction model (it tracks the planner's streamed reference trajectory).

This replaces the separate two/three/four launch files: num_drones is an
argument. The node list is built inside an OpaqueFunction because a
LaunchConfiguration is an unresolved substitution at generate_launch_description()
time -- you cannot loop over it there. OpaqueFunction defers until launch, where
.perform(context) gives the actual integer.

Run Gazebo first, then this, matching num_drones to the world:
    gz sim simulation_assets/two_rigid_short.sdf   -v 4 -r
    ros2 launch controller_quad_load mpc_quad_load_launch.py num_drones:=2

    gz sim simulation_assets/three_rigid_short.sdf -v 4 -r
    ros2 launch controller_quad_load mpc_quad_load_launch.py num_drones:=3

    gz sim simulation_assets/four_rigid_short.sdf  -v 4 -r
    ros2 launch controller_quad_load mpc_quad_load_launch.py num_drones:=4

All three *_rigid_short worlds share the same geometry (0.5 m rigid rods at
45 deg, 0.4 kg payload), so cable_len/load_mass need no override. For the longer
worlds:
    two_rigid.sdf / four_rigid.sdf  cable_len:=0.707
    three_soft.sdf                  cable_len:=0.6
    three_soft_paper.sdf            cable_len:=1.0 load_mass:=0.1

The online load-cable OCP (Sun et al. 2025) generates the references. The default
config is a GROUND START: the drones sit on the floor, arc-sweep the rigid rods up
to handover_elev_deg, then the OCP takes over and lifts the load, e.g.
    gz sim simulation_assets/two_rigid_ground.sdf -v 4 -r
    ros2 launch controller_quad_load mpc_quad_load_launch.py num_drones:=2
For an elevated taut world instead, set start_taut:=true handover_elev_deg:=0 to
skip the creep and hand straight to the OCP.

start_taut:=true ALSO works as a straight "skip the takeoff stage" experiment on a
GROUND world: the planner hands to the coupled OCP on its first tick instead of
arc-sweeping the rods up first, so the controller flies the drones off the ground
itself. The lift ramp and handover_settle_s still happen -- only the creep phase is
removed. See the start_taut argument below for what that risks.

Fleet control:
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: ARM}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: TAKEOFF}"
    ros2 topic pub -t 3 /fleet/command std_msgs/msg/String "{data: LAND}"
(-t 3 rather than --once: each `ros2 topic pub` call is a fresh publisher that
races discovery, and every handler here is idempotent.)

RUN ORDER: rviz_quad_load_launch.py FIRST (it owns the clock/pose bridges, the
mocap emulators and fleet_viz), then this. Bringing RViz up first lets you
confirm the fleet is present and its poses are live before you fly it. This
launch does not duplicate those nodes, so on its own the trackers get no pose.

NOTE ON FLEET SIZE: per-drone cable tension is mg/(n sin45), so it RISES as
drones come off -- n=2 needs ~14 deg of standing tilt vs ~8 deg at n=4. n=2 also
leaves the payload free to rotate about the line joining its two attach points,
a DOF no drone arrangement can control (see two_rigid_short.sdf notes).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

PARENT_MODEL = 'lift_system'


def _args():
    return [
        # Fleet size. MUST match the world SDF: the planner sizes the attach ring
        # and divides the load tension by this, so a mismatch mis-scales every
        # drone's feedforward.
        DeclareLaunchArgument('num_drones', default_value='2'),
        # must match the tether length in the world SDF -- the *_rigid_short
        # worlds use rigid 0.5 m rods, and a mismatch here commands a formation
        # radius the tethers physically can't reach (drones fight the rod).
        DeclareLaunchArgument('cable_len', default_value='0.5'),
        # true = SKIP THE CREEP PHASE. The planner hands straight to the coupled OCP
        # on its first tick (planner_node: phase 'creep' -> 'planner' immediately),
        # rather than first sweeping the rods up to handover_elev_deg. Intended for
        # elevated taut worlds, but it is also the switch for "no takeoff stage, let
        # the controller fly it off the ground" on a *_ground world.
        # WHAT REMAINS: the lift ramp (lift_ramp_vel -> target_z) and
        # handover_settle_s. Only the creep is removed.
        # WHAT IT RISKS on a ground start: the creep exists because the OCP assumes
        # TAUT cables. From the ground the OCP immediately commands the full 45deg
        # formation, which on rigid rods is a large position step the drones cannot
        # take directly (they are constrained to an arc pivoting about grounded attach
        # points), and while the payload is still resting the cable feedforward is
        # gated off -- so the no-integrator tracker is asked for a big move with no
        # tension feedforward. That is the failure the creep was written to avoid.
        # DEFAULT TRUE: verified in sim on three_rigid_ground (run 20260803_115853),
        # which flew clean -- 3 mm steady-state payload tracking. Set false to get
        # the arc creep back if a different world or geometry needs it.
        DeclareLaunchArgument('start_taut', default_value='true'),
        # GROUND-START rigid worlds only (three_rigid_ground.sdf). Degrees of
        # cable elevation the drones must sweep up to -- along the rod's arc,
        # pivoting about the grounded attach points -- before the planner takes
        # over. A rigid rod is always full length, so the usual slack->taut
        # handover fires instantly at ~6 deg where tension is ~13 N/drone. 0 =
        # off (elevated or soft worlds).
        DeclareLaunchArgument('handover_elev_deg', default_value='45.0'),
        # Seconds to hold the latched config after handover before lifting.
        # Ground starts want ~2.0 so the reference step and the payload breaking
        # ground don't land in the same cycle. 1.0 lets the coupled taut-air-start
        # solver converge on the taut hover before the climb ramp begins (smoother
        # takeoff); 0 = off.
        DeclareLaunchArgument('handover_settle_s', default_value='0.75'),
        # payload mass in the world SDF.
        DeclareLaunchArgument('load_mass', default_value='0.4'),
        DeclareLaunchArgument('target_z', default_value='0.6'),
        # HOLD test: lift_ramp_vel:=0.0 (no lift, just hold the taut config).
        # 0.22 is a brisk-but-trackable climb rate; drop toward 0.12 for a gentler lift.
        DeclareLaunchArgument('lift_ramp_vel', default_value='0.22'),
        # LAND descent rate (separate from the slow takeoff lift_ramp_vel).
        DeclareLaunchArgument('land_vel', default_value='0.20'),
        # Cable compensation. ON is the correct flight config: the cable pulls
        # each drone inward-and-down, so it must hold an outward tilt just to
        # stay put. Only zero these for a deliberate A/B:
        #   cable_ff_scale:=0.0  cable-blind prediction model
        #   attitude_ff:=false   level attitude reference (throttle FF kept)
        DeclareLaunchArgument('cable_ff_scale', default_value='1.0'),
        DeclareLaunchArgument('attitude_ff', default_value='true'),
        # 'model' = planner open-loop t*s/m cable term; 'measured' = IMU f_ext.
        DeclareLaunchArgument('cable_source', default_value='model'),
        # payload counts as resting (cable term zeroed) at/below this z.
        DeclareLaunchArgument('payload_rest_z', default_value='-0.1'),
        # seconds to spool throttle up at takeoff; 0 = instant. 0.5 eases the
        # applied throttle on (raised-cosine from TAKEOFF_SPOOL_FLOOR*u to u) and
        # completes inside handover_settle_s, so it smooths the idle->hover
        # engagement without scaling throttle during the climb (which would starve
        # the lift). See TAKEOFF_SPOOL_FLOOR in controller_mpc.py.
        DeclareLaunchArgument('takeoff_spool_s', default_value='0.5'),
        # Thrust accel per unit throttle the MPC assumes (kT). The sim plant is
        # actually QUADRATIC -- a(u) = c*u^2 from the SDF motor model -- so no
        # single kT is right everywhere; see thrust_quad_c below. 24.0 also matches
        # the hardware airframe, which is the point of the current motorConstant.
        # NOTE: raising this toward the hover value STARVES takeoff -- at the low
        # throttle on the stands the quadratic plant's effective kT (c*u) is well
        # below its hover value, so a hover-tuned constant under-thrusts and the
        # drones slide off and drop. Keeping it a little LOW is deliberate: the
        # resulting over-thrust is the pop that breaks them off the stands. The
        # residual hover error is handled by the schedule/estimator once airborne,
        # NOT by raising this constant.
        # 30.0 (was 24.0, the HARDWARE value) because this is a SIM launch and with
        # motorConstant=0.62e-06 the sim's true loaded-hover kT is ~33. It is now also
        # the kT ESTIMATOR'S SEED and the centre of its +/-kt_max_deviation band, so a
        # hardware-accurate 24 put the truth (33) outside the band (20.4..27.6) and the
        # estimate pinned at 27.60 forever. 30 keeps 33 inside the band (25.5..34.5)
        # while still sitting low enough for a mild (33/30)^2 = 1.2x takeoff pop.
        # The hardware launches (real_*.py) keep their own 24.
        DeclareLaunchArgument('thrust_ratio', default_value='30.0'),
        # Quadratic-plant coefficient c in a(u)=c*u^2. DERIVED FROM THE SDF, and it
        # MUST be re-derived whenever motorConstant changes:
        #     c = 4 * motorConstant * maxRotVelocity^2 / mass
        #       = 4 * 0.62e-06 * 4631^2 / 0.6  =  88.6
        # It was 203.0, which was correct for the old motorConstant 1.42e-06. After
        # that was lowered to 0.62e-06 ("match real world") the stale 203 made the
        # airborne schedule compute kT = clip(203*0.371, 24, 55) = 55 -- clipped at
        # the ceiling and ~67% above the true ~33 -- so the drone commanded a third
        # of the thrust it needed and dropped, right after the fixed 24 had
        # over-thrust it off the stand. That two-stage mismatch IS the takeoff
        # bounce. Set 0.0 to disable the schedule and let the adaptive estimator
        # below own kT instead (it measures c*u directly, no SDF constant needed).
        DeclareLaunchArgument('thrust_quad_c', default_value='88.6'),
        # ADAPTIVE kT. Every drone measures its own thrust ratio in flight from
        # its IMU (body-z specific force minus the planner's known cable pull,
        # divided by applied throttle) and runs a scalar Kalman filter on it, so
        # the MPC stops depending on thrust_ratio/thrust_quad_c being right. Each
        # tracker prints a ~1 Hz line: est vs the schedule vs what the MPC is
        # flying. adaptive_thrust_feedback:=false = SHADOW mode (print/log only,
        # the MPC keeps the open-loop schedule) -- use it for the first run to
        # check the numbers before letting the estimate drive the thrust model.
        DeclareLaunchArgument('adaptive_thrust_ratio', default_value='true'),
        DeclareLaunchArgument('adaptive_thrust_feedback', default_value='true'),
        # Estimator backend:
        #   'ukf' (default) -- the controller_ukf method. 19-state UKF over
        #     [pose(13) | 6 dynamics params], sigma points propagated through the
        #     real model and corrected against MOCAP. No IMU dependence at all, and
        #     drag/tau mismatch is absorbed by their own states instead of by kT.
        #     Costs ~4.5 ms/cycle (39 integrator calls, ~22% of a 20 ms cycle).
        #   'none' -- no estimation: the thrust_quad_c schedule, else thrust_ratio.
        # NOTE a CONSTANT cable-tension bias moves kT ~7% per 30% of tension error.
        # That is an identifiability limit at constant throttle, not something the
        # filter can fix; kt_max_deviation is the practical guard.
        DeclareLaunchArgument('thrust_ratio_estimator', default_value='ukf'),
        # kT process-noise variance: how fast the estimate may move. Tuned on the
        # assumption the SEED IS CLOSE (thrust_ratio from the SDF motor model / a bench
        # test), so the metric is WANDER, not convergence. At 1e-3 the estimate holds
        # to 0.3% through a +/-35% tension swing and still corrects a 10%-off seed in
        # ~13 s. Raise toward 1e-2 only if the seed cannot be trusted.
        DeclareLaunchArgument('ukf_q_kt', default_value='0.001'),
        # HARD BAND around the seed, as a fraction: kT is confined to
        # thrust_ratio*(1 +/- this). A disturbance the filter cannot explain can at
        # worst nudge kT, never walk it somewhere absurd. Applies to BOTH backends.
        # 0 disables it (raw kt_min/kt_max only).
        DeclareLaunchArgument('kt_max_deviation', default_value='0.15'),
        # ESTIMATOR SEED / band centre -- the drone's ACTUAL hover kT, decoupled from
        # thrust_ratio (which is the takeoff constant and is kept LOW on purpose).
        # 33.0 = 88.6*0.376, the operating-point secant gain the schedule measures in
        # flight. Seeding at the takeoff-safe 30 left the estimate ~10% low, the MPC
        # over-throttled and the load overshot its target by ~0.18 m. 0 = use
        # thrust_ratio (the old coupled behaviour).
        DeclareLaunchArgument('kt_seed', default_value='33.0'),
        # LEARN-THEN-LOCK: estimate kT for this many seconds after becoming airborne
        # (the lift -- near-hover, well modelled, load going straight up), then FREEZE
        # it for the rest of the flight. kT is a property of the airframe and battery,
        # not of the trajectory, so there is nothing to track once it is known; and a
        # frozen estimate cannot be dragged around by cable-tension error or payload
        # swing during the trajectory. Freezing also stops kt_input, so the estimator
        # nodes idle and hand their cores back exactly when the trajectory needs them.
        # 0 = never freeze (estimate for the whole flight).
        DeclareLaunchArgument('kt_freeze_after_s', default_value='10.0'),
        # UKF rate. 50 = every control cycle (as controller_ukf). Lower it for
        # headroom; the integrator step is set at runtime, so no rebuild is needed.
        DeclareLaunchArgument('ukf_rate_hz', default_value='10.0'),
        # seconds between the per-drone kT reports; 0 = silent.
        DeclareLaunchArgument('kt_print_period_s', default_value='1.0'),
        # Auto slot assignment (coupled mode). OFF by default so the sim behaves as
        # before (slot i == drone i, matching the SDF spawn order). Set true to test
        # the real-world behaviour: each drone is matched to the nearest nominal ring
        # slot at the first solve, so spawn order/labelling stops mattering. In a
        # normally-ordered sim world this resolves to the identity (a no-op) -- to
        # actually exercise the reorder, spawn the drones out of azimuth order.
        DeclareLaunchArgument('auto_slot_assign', default_value='true'),
        # LOAD reference after the lift tops out: 'hover', 'line_x' (continuous
        # back-and-forth shuttle), 'circle', 'fig_8' (figure-eight lemniscate),
        # 'spin' (circle + the load yaws one full turn, so the formation also
        # rotates about the payload). circle/fig_8/spin use traj_radius; line_x uses
        # traj_distance; all use traj_speed. Keep traj_speed slow -- lateral accel
        # is fed forward via the flatness cable ref but tracking still lags at speed.
        DeclareLaunchArgument('load_traj', default_value='hover'),
        DeclareLaunchArgument('traj_speed', default_value='0.6'),
        DeclareLaunchArgument('traj_distance', default_value='1.0'),
        DeclareLaunchArgument('traj_radius', default_value='0.5'),
    ]


def launch_setup(context, *args, **kwargs):
    # Resolve num_drones to a real int -- this is the whole reason for
    # OpaqueFunction. Everything else can stay a substitution.
    n = int(LaunchConfiguration('num_drones').perform(context))
    if n < 1:
        raise RuntimeError(f'num_drones must be >= 1, got {n}')
    drone_names = [f'x3_drone{i}' for i in range(n)]

    f = lambda name: ParameterValue(LaunchConfiguration(name), value_type=float)
    b = lambda name: ParameterValue(LaunchConfiguration(name), value_type=bool)

    nodes = [SetParameter(name='use_sim_time', value=True)]

    # NOTE: the clock bridge, the drone/payload POSE bridges and the mocap
    # emulators are NOT here -- they belong to rviz_quad_load_launch.py, which
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
                         'parent_model': PARENT_MODEL}]))
        # per-drone CABLE-AWARE MPC tracker — tracks the planner reference
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
                         'thrust_quad_c': f('thrust_quad_c'),
                         'adaptive_thrust_ratio': b('adaptive_thrust_ratio'),
                         'adaptive_thrust_feedback': b('adaptive_thrust_feedback'),
                         'thrust_ratio_estimator':
                             LaunchConfiguration('thrust_ratio_estimator'),
                         'kt_max_deviation': f('kt_max_deviation'),
                         'kt_seed': f('kt_seed'),
                         'kt_freeze_after_s': f('kt_freeze_after_s'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            output='screen'))
        # OUT-OF-LOOP kT estimator, one process per drone. The parameter UKF is 39
        # acados propagations; run inside the tracker's 50 Hz timer it measured
        # 25-150 ms and stalled the control loop. Here it cannot: if it falls behind
        # it just produces estimates less often. Skipped unless the ukf backend is
        # selected -- the other backends need no helper process.
        nodes.append(Node(
            package='controller_quad_load', executable='kt_estimator',
            name=f'kt_estimator_{i}',
            parameters=[{'drone_id': i,
                         'thrust_ratio': f('thrust_ratio'),
                         'kt_seed': f('kt_seed'),
                         'kt_max_deviation': f('kt_max_deviation'),
                         'ukf_q_kt': f('ukf_q_kt'),
                         'ukf_rate_hz': f('ukf_rate_hz'),
                         'kt_print_period_s': f('kt_print_period_s')}],
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration('thrust_ratio_estimator'), "' == 'ukf' and '",
                 LaunchConfiguration('adaptive_thrust_ratio'), "'.lower() == 'true'"])),
            output='screen'))

    # ── Central fleet manager ──────────────────────────────────────────────
    nodes.append(Node(
        package='controller_quad_load', executable='main', name='central_controller',
        parameters=[{'num_drones': n}], output='screen'))

    # ── Centralized cable-suspended load planner ───────────────────────────
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
