"""
planner_node.py
---------------
Centralized planner for a cable-suspended load. Runs at PLANNER_HZ and publishes
a reference trajectory per drone on /drone_{i}/reference_trajectory, consumed by
the per-drone tracker in controller_quad_load.

Wire format (Float64MultiArray), 12 fields per node, world frame:
    [n_nodes, dt, px,py,pz, vx,vy,vz, ax,ay,az, cx,cy,cz, ...]
    a_i  required specific thrust acceleration (f_i/m_i): magnitude sets the
         tracker throttle, direction sets its desired yaw-free attitude.
    c_i  cable tension acceleration t_i*s_i/m_i, added to the tracker's
         prediction model so its feedback stops fighting the cable.

Solves the load-cable OCP (planner_ocp.py) online, pinning node 0 to the measured
state. This is the paper's method (Sun et al. 2025). x_init is built from mocap --
load pose/twist, cable directions s_i AND cable angular velocities r_i are all
measured; only the unobservable higher cable states (rd_i, rdd_i) and tensions
(t_i, td_i) are resampled from the previous solution (paper Fig 8). The fleet
creeps up from the ground (start_taut=false, handover_elev_deg>0), sweeping the
rigid rods up until the cables are taut, then the OCP takes over; an elevated
start_taut world skips the creep and hands over immediately.

Geometry must match the world SDF (see generate_rigid_world.py).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from interfaces.msg import MotionCaptureState

from .load_cable_dynamics import LoadCableDynamics, LOAD_DIM, CABLE_DIM
from .geometry import (quat_to_rot_np, attach_points, nominal_cable_dirs,
                       azimuth_slot_assignment, yaw_from_quat)
from .load_trajectory import LoadTrajectory
from .planner_solver import PlannerSolver
from .params import PlannerConfig
from .creep_controller import CreepController
from .reference_builder import ReferenceBuilder
from utility_objects.data_logger import run_log_dir, write_params, node_params
from utility_objects.run_context import log_base_dir

# Physical constants baked into the load-cable model (not ROS params). Geometry and
# mode params (num_drones, cable_len, load_mass, ...) live in params.PlannerConfig.
# Sized for the original 0.4 kg payload. Inertia scales with mass for a body of
# fixed geometry, so it does NOT track the load_mass ROS param -- dropping
# load_mass to 0.1 without scaling these leaves the model ~4x over-stiff in
# rotation. Scale by (load_mass / 0.4) for a same-size lighter payload, or
# recompute from the real payload's dimensions.
LOAD_INERTIA  = [1.67e-3, 1.67e-3, 3.33e-3]
DRONE_MASS    = 0.6
PLANNER_HZ    = 10.0

# logs/<LOG_PKG>/<node>_<ts>/params.json -- the same tree the per-drone trackers log
# into, so a run's planner configuration sits alongside its per-drone CSVs.
LOG_PKG       = 'controller_quad_load'

# Lift-ramp easing: shape the rate 0 -> lift_ramp_vel -> 0 rather than stepping,
# since the velocity feedforward is passed straight through and a step there is
# taken by the drones as a jolt. For a 0.475 m climb at 0.20 m/s this cuts peak
# accel 2.00 -> 0.59 m/s^2 and arrives at ~0.01 m/s, costing ~1.5 s.
LIFT_SOFT_S   = 1.2           # s to ease in
LIFT_SOFT_D   = 0.08          # m to ease out over
LIFT_SOFT_MIN = 0.12          # floor on the shape factor, so the climb actually
                              # terminates instead of asymptoting at the target

# LAND ends on touchdown (drones stop following the descending reference), not at
# an absolute height: the fleet may be over a takeoff platform rather than open
# ground, making any fixed threshold unreachable. See _touchdown_stalled.
LAND_STALL_FRAC = 0.25        # descent slower than this fraction of commanded
                              # counts as stalled
LAND_STALL_S  = 1.0           # s the stall must persist
LAND_MIN_DESCENT = 0.05       # m the fleet must actually drop first
LAND_GRACE_S  = 1.5           # s to ignore the stall test after LAND, while the
                              # drones are still catching up to the new reference
LAND_MAX_DROP = 1.50          # m safety floor below the handover height

# Cable-FF soft-start. The payload sits on the GROUND through the handover settle,
# so stepping the full tension feedforward on at handover makes the drones lurch to
# absorb a pull that isn't real yet (and the OCP then tracks that thrash). Ease the
# published a_cable in over FF_EASE_S once the LIFT starts -- clocked on the
# planner's own lift schedule, not the measured load height, so unlike the old
# airborne height-gate it can never deadlock (the lift clock advances regardless).
FF_EASE_S = 1.0

LIFT_STEP     = 0.04          # m max commanded climb above current load z. Kept
                              # small: a large lead builds climb speed and
                              # overshoots the taut transition, spiking tension
                              # past the drones' thrust authority.

# Cable tautness gate. Cables spawn slack, so feeding predicted tension while one
# is loose makes the tracker over-estimate the pull and lurch. Scale the published
# cable acceleration by how taut the cable measures right now (drone->attach
# distance vs cable length). Raise LO_FRAC toward 1.0 to feed tension in later.
CABLE_TAUT_LO_FRAC = 0.85
CABLE_TAUT_HI_FRAC = 1.00


class LoadPlanner(Node):
    def __init__(self):
        super().__init__('load_planner')

        # ROS params (see params.PlannerConfig). Copied onto self so the rest of the
        # planner reads plain self.<name>.
        cfg = PlannerConfig(self)
        cfg.log(self.get_logger())
        self.n = cfg.n
        self.cable_len = cfg.cable_len
        self.attach_radius = cfg.attach_radius
        self.attach_z = cfg.attach_z
        self.load_mass = cfg.load_mass
        self.handover_elev_deg = cfg.handover_elev_deg
        self.handover_settle_s = cfg.handover_settle_s
        self.start_taut = cfg.start_taut
        self.target_z = cfg.target_z
        self.lift_ramp_vel = cfg.lift_ramp_vel
        self.auto_slot_assign = cfg.auto_slot_assign
        self.load_traj = cfg.load_traj
        self.traj_speed = cfg.traj_speed
        self.traj_distance = cfg.traj_distance
        self.traj_radius = cfg.traj_radius
        self.land_vel = cfg.land_vel

        # Flight-sequence runtime state (not params).
        self._settle_left = 0.0
        self.traj_t = 0.0            # elapsed lateral-trajectory time (post-hover)
        self.descending = False      # latched once the lateral trajectory closes
        self._land_to_ground = False # LAND command: descend all the way to ground
        self._landed = False         # descent finished, load back at start height
        self._lift_vel = 0.0         # signed vertical velocity of the lift target
        # touchdown detector state (see _touchdown_stalled)
        self._land_prev_max_z = None
        self._land_stall_ct = 0
        self._land_cycles = 0

        # Lateral load-reference trajectory generator (line_x / circle / fig_8 /
        # spin). The trajectory CLOCK traj_t stays here and is advanced in _plan.
        self.traj = LoadTrajectory(self.load_traj, self.traj_speed,
                                   self.traj_distance, self.traj_radius)

        # Attach ring on the payload body + nominal 45 deg cable directions (the
        # flatness s_i reference at hover, tilted per node by the load accel in
        # _yref_at). Both derived from the fleet size and attach geometry.
        self.rho = attach_points(self.n, self.attach_radius, self.attach_z)
        self._s_nom = nominal_cable_dirs(self.rho, 45.0)

        self.dyn = LoadCableDynamics(
            self.n, self.load_mass, LOAD_INERTIA, [self.cable_len] * self.n,
            self.rho, DRONE_MASS)
        # The OCP wrapper builds (or loads a cached) acados solver for this geometry
        # and owns the reference-extraction functions + warm-start state (last_X).
        self.solver = PlannerSolver(self.dyn, self.get_logger())
        # Pre-built OCPs by fleet size, for handing back after a reconfiguration.
        # Populated by prebuild_solvers(); the current size is registered here so a
        # hand-back to the original n is a swap like any other.
        self._solvers = {self.n: (self.dyn, self.solver, self.rho)}
        self.N = self.solver.N
        self.dt = self.solver.dt
        # Phase-1 takeoff (soft/arc creep + handover decision), owns its own creep
        # state. Fed the live measured state each tick and publishes the creep refs
        # through _publish_ref (the trackers see these until the OCP takes over).
        self.creep = CreepController(
            self.n, self.rho, self.cable_len, self.N, self.dt, self.dyn.g,
            self.handover_elev_deg, PLANNER_HZ, self._drone_at, self._publish_ref,
            self.get_logger())
        # Builds the per-node OCP tracking reference from the lift schedule + load
        # trajectory (fed the schedule via refs.update() before each planner solve).
        self.refs = ReferenceBuilder(self.dyn, self.n, self._s_nom, self.dt, self.traj)

        # state
        self.load_state = None                 # [p(3), q(4 wxyz), v(3), w(3)]
        self.drone_pos = [None] * self.n
        self.drone_vel = [None] * self.n       # world velocity from mocap twist
        # OCP slot i -> physical drone slot2drone[i]. Identity until (optionally)
        # reassigned by azimuth on the first solve (see _assign_slots).
        self.slot2drone = list(range(self.n))
        self._slots_assigned = False
        # Load yaw the rig was placed at, latched on the first planner tick
        # (_latch_yaw_datum). 0.0 until then, which is the old world-aligned
        # behaviour and is correct for a payload that really is at yaw 0.
        self.psi0 = 0.0
        self._yaw_datum_latched = False
        # NB: the OCP warm-start state (last_X / recover) lives on self.solver.
        self.hover_xy = None                   # captured load x,y for the reference
        self._ff_t = 0.0                       # cable-FF soft-start clock (see _plan)
        self.phase = 'creep'                   # 'creep' (slow rise to taut) -> 'planner'
        self.takeoff_seen = False              # gate the lift ramp on TAKEOFF (see below)
        self.lift_z0 = None                    # load height latched at handover
        self.lift_progress = 0.0               # ramped lift above lift_z0 (m)
        self._lift_t = 0.0                     # s since the lift ramp started

        # subs
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_cb, 5)
        # /fleet/step is 0 before TAKEOFF and increments once flying, so step > 0
        # is the cue that it is safe to start the lift ramp (see _plan).
        self.create_subscription(
            Int32, '/fleet/step',
            lambda msg: setattr(self, 'takeoff_seen',
                                self.takeoff_seen or msg.data > 0), 5)
        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda msg, k=i: self._drone_cb(msg, k), 5)
        # /fleet/command (ARM|TAKEOFF|DISARM|ESTOP|LAND). Only LAND is acted on:
        # stop the lateral trajectory and descend. The central controller
        # disarms once we report done.
        self.create_subscription(
            String, '/fleet/command', self._fleet_command_cb, 10)

        # pubs
        self.ref_pub = [self.create_publisher(
            Float64MultiArray, f'/drone_{i}/reference_trajectory', 5)
            for i in range(self.n)]
        # True once the LAND descent finishes, so the central controller disarms.
        self.landed_pub = self.create_publisher(Bool, '/fleet/landed', 1)
        # Desired load position [x, y, z] at node 0. Logged by the drone-0
        # tracker so plot_run.py can overlay payload desired vs actual.
        self.load_ref_pub = self.create_publisher(
            Float64MultiArray, '/payload/desired_position', 5)
        # Same reference as a full horizon Path, for RViz alongside each drone's
        # /drone_N/mpc_plan.
        self.load_plan_pub = self.create_publisher(
            Path, '/payload/mpc_plan', 5)

        self.create_timer(1.0 / PLANNER_HZ, self._plan)
        # Record the run's configuration. Written here AND re-written at the end of a
        # subclass's __init__ (see _dump_run_params), so the file always reflects the
        # full parameter set of whichever node actually flew.
        self._run_log_dir = None
        self._dump_run_params()
        self.get_logger().info('[planner] ready, waiting for mocap...')

    def _dump_run_params(self):
        """Write logs/<pkg>/<node>_<ts>/params.json with every declared parameter, so a
        run says what it was configured with instead of having to be reverse-engineered
        from the flown trajectory. Safe to call more than once -- the directory is made
        on the first call and the file is overwritten after that, which is how a
        subclass folds in the parameters it declares after super().__init__()."""
        try:
            if self._run_log_dir is None:
                # Absolute results root (see utility_objects.run_context) so the
                # planner's params.json sits beside the trackers' CSVs instead of
                # wherever the launch happened to be started from.
                self._run_log_dir = run_log_dir(LOG_PKG, self.get_name(),
                                                base_dir=log_base_dir())
            write_params(self._run_log_dir, node_params(self, {
                'node': self.get_name(), 'planner_hz': PLANNER_HZ,
                'num_drones': self.n, 'horizon_N': self.N, 'node_dt': self.dt,
                'phase_at_start': self.phase}))
        except Exception as e:                      # never break the flight for a log
            self.get_logger().warn(f'[planner] could not write params.json: {e}')

    # Mocap callbacks
    def _payload_cb(self, msg: MotionCaptureState):
        p = msg.pose.position
        o = msg.pose.orientation
        lv = msg.twist.linear
        av = msg.twist.angular
        self.load_state = np.array([
            p.x, p.y, p.z, o.w, o.x, o.y, o.z,
            lv.x, lv.y, lv.z, av.x, av.y, av.z])
        if self.hover_xy is None:
            self.hover_xy = (p.x, p.y)

    def _drone_cb(self, msg: MotionCaptureState, i):
        p = msg.pose.position
        lv = msg.twist.linear
        self.drone_pos[i] = np.array([p.x, p.y, p.z])
        self.drone_vel[i] = np.array([lv.x, lv.y, lv.z])

    def _fleet_command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        # Log every command before filtering: LAND is the only one acted on, so
        # otherwise a non-matching subscription looks identical to a LAND that
        # was delivered and filtered out.
        self.get_logger().info(
            f'[planner] /fleet/command received: {cmd!r} '
            f'(phase={self.phase} takeoff_seen={self.takeoff_seen} '
            f'descending={self.descending} land_to_ground={self._land_to_ground})')
        if cmd != 'LAND':
            return
        if self.phase != 'planner' or not self.takeoff_seen:
            self.get_logger().warn('[planner] LAND ignored - not flying yet')
            return
        if self._land_to_ground:
            # A repeat LAND means the first didn't take, usually because the
            # central controller missed it while we heard it. If we already
            # touched down, re-announce: the one-shot /fleet/landed is long gone
            # and no retry could otherwise complete the handshake.
            if self._landed:
                self.landed_pub.publish(Bool(data=True))
                self.get_logger().info(
                    '[planner] LAND repeated after touchdown — re-announcing '
                    '/fleet/landed')
            else:
                self.get_logger().info('[planner] LAND already in progress')
            return
        # descending alone must not gate this: a closing lateral trajectory
        # latches it too, but that auto-descent stops at the handover height and
        # never sets _land_to_ground, so /fleet/landed is never published and the
        # fleet never disarms. LAND upgrades an auto-descent to a full one.
        self.descending = True
        self._land_to_ground = True
        self._landed = False
        # Arm the touchdown detector fresh: this may be upgrading a descent
        # already in progress, so stale stall state must not carry over.
        self._land_prev_max_z = None
        self._land_stall_ct = 0
        self._land_cycles = 0
        self._land_start_z = None
        self.get_logger().info('[planner] LAND - descending to the floor')

    def _touchdown_stalled(self):
        """True once every drone has stopped descending, i.e. touched down.

        Called only while a LAND descent drives the reference down at land_vel.
        A drone no longer following that command has hit something solid, which
        is the only landing test that holds when the fleet is not above its
        takeoff points. The stall must persist LAND_STALL_S so tracking lag or a
        swinging load doesn't read as touchdown.
        """
        if any(d is None for d in self.drone_pos):
            return False
        max_dz = max(d[2] for d in self.drone_pos)
        # Arm the stall test only after the fleet has actually descended a real
        # distance. The grace period alone isn't enough: if the tracker lagged
        # longer than LAND_GRACE_S the drones would still be stationary when the
        # test armed, misreading "not moving yet" as "landed" at altitude.
        if self._land_start_z is None:
            self._land_start_z = max_dz
        self._land_cycles += 1
        if (self._land_cycles < int(LAND_GRACE_S * PLANNER_HZ)
                or max_dz > self._land_start_z - LAND_MIN_DESCENT):
            self._land_prev_max_z = max_dz
            return False
        prev = self._land_prev_max_z
        self._land_prev_max_z = max_dz
        if prev is None:
            return False
        # expected drop this cycle if the drones were tracking the reference
        expected = self.land_vel / PLANNER_HZ
        if (prev - max_dz) < expected * LAND_STALL_FRAC:
            self._land_stall_ct += 1
        else:
            self._land_stall_ct = 0
        return self._land_stall_ct >= int(LAND_STALL_S * PLANNER_HZ)

    def _drone_at(self, i):
        """Measured position of the physical drone occupying OCP slot i (identity
        unless auto_slot_assign remapped it)."""
        return self.drone_pos[self.slot2drone[i]]

    def _assign_slots(self):
        """Match each physical drone to the nearest nominal azimuth slot around the
        load (see geometry.azimuth_slot_assignment), so the drones can be placed in
        the ring in any order. Relabels I/O only -- the OCP is unchanged.

        Matched in the LOAD frame: the slots ARE the attach points, which rotate with
        the payload. Matching in the world frame instead mismatched every drone to a
        neighbouring attach point as soon as the payload was placed past half a slot
        pitch, which made the first solve QP-infeasible (see the function's docstring)."""
        self.slot2drone = azimuth_slot_assignment(
            self.drone_pos, self.load_state[0:2], self.n, load_yaw=self.psi0)
        self._slots_assigned = True
        self.get_logger().info(
            f'[planner] auto slot assignment (slot->drone): {self.slot2drone} '
            f'(load yaw datum {np.degrees(self.psi0):+.1f} deg)')

    def _latch_yaw_datum(self):
        """Latch the payload's measured yaw as the reference datum, once, before the
        first solve. Everything downstream -- the slot matching, the nominal cable
        ring in yref_at/hold_yref, and the load attitude reference q_ref -- is
        expressed about it, so the fleet holds the yaw the rig was PLACED at instead
        of rotating the payload onto world +x on takeoff."""
        self.psi0 = yaw_from_quat(self.load_state[3:7])
        self.refs.set_yaw_datum(self.psi0)
        self._yaw_datum_latched = True
        self.get_logger().info(
            f'[planner] load yaw datum latched at {np.degrees(self.psi0):+.1f} deg')

    # Plan step
    def _plan(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return

        # Latch the placement yaw datum first: the slot matching below is expressed
        # about it, as is every reference the builder produces.
        if not self._yaw_datum_latched:
            self._latch_yaw_datum()

        # Match drones to nominal slots once, now that every pose is in.
        if self.auto_slot_assign and not self._slots_assigned:
            self._assign_slots()

        self._publish_load_desired()

        # Phase 1: creep takeoff. The planner assumes taut cables, but running the
        # lift through the ~0.4 m of slack makes the drones overshoot and snap the
        # cable taut, which the open-loop tracker cannot ride. Creep up slowly
        # instead so the slack->taut transition is gentle and tension never spikes
        # past the drones' thrust authority. start_taut worlds skip this.
        if self.phase == 'creep' and self.start_taut:
            self._enter_planner_phase('start_taut')

        gates = [self._cable_taut_gate(i) for i in range(self.n)]
        if self.phase == 'creep':
            handover, reason = self.creep.step(self.load_state, self.drone_pos, gates,
                                               self.takeoff_seen)
            # Keep the OCP warm on the live measured config (discarding its horizon,
            # the trackers stay on the creep refs) so the handover has a warm start.
            self._prime_solver()
            if handover:
                self._enter_planner_phase(reason)
            return

        # Vertical phases: ascend to target_z, run the lateral trajectory, then
        # descend once it closes. The ramp must not advance before TAKEOFF: the
        # planner runs from first mocap while the drones sit idle, so
        # lift_progress would run metres above them and yank them up the instant
        # they arm. A position-based liftoff test can't substitute, since in the
        # elevated start_taut world the drones spawn airborne and never rise
        # above spawn to trip it.
        prev_progress = self.lift_progress
        if self.takeoff_seen and self._settle_left > 0.0:
            # post-handover hold: reference frozen at the latched config, load
            # still grounded so the FF gate stays at 0. Nothing accumulates.
            self._settle_left -= 1.0 / PLANNER_HZ
            if self._settle_left <= 0.0:
                self.get_logger().info(
                    '[planner] handover settle complete — starting lift ramp')
        elif self.takeoff_seen:
            # Cable-FF soft-start clock: advances only once the lift is active (NOT
            # during the grounded settle above), so the tension FF eases in as the
            # load breaks ground instead of stepping on. Applied in _publish_refs.
            self._ff_t += 1.0 / PLANNER_HZ
            if self.descending:
                # Auto-descent stops at the handover height (lift_progress -> 0).
                # LAND keeps driving the reference below that until every drone
                # is on the floor: the reference goes underground, the drones
                # just stop when they hit it. Only tracks cleanly if the drones
                # follow closely; a mistuned thrust_ratio floats the load ~1 m
                # above its reference and the descent goes unstable.
                floor = -LAND_MAX_DROP if self._land_to_ground else 0.0
                rate = self.land_vel if self._land_to_ground else self.lift_ramp_vel
                self.lift_progress = max(
                    floor, self.lift_progress - rate / PLANNER_HZ)
                if self._land_to_ground:
                    # Touchdown by stall, not absolute height: over a takeoff
                    # platform a fixed threshold is unreachable, so the descent
                    # would only end when lift_progress bottoms out, grinding the
                    # drones into the platform for ~10 s. Works either way.
                    done = self._touchdown_stalled() \
                        or self.lift_progress <= floor + 1e-6
                else:
                    done = self.lift_progress <= 1e-6
                if done and not self._landed:
                    self._landed = True
                    if self._land_to_ground:
                        self.landed_pub.publish(Bool(data=True))
                    self.get_logger().info(
                        '[planner] descent complete — drones on the floor')
                elif self._landed and self._land_to_ground:
                    # Keep announcing until the fleet disarms. A single publish is
                    # lost if the controller missed the original LAND (its
                    # callback drops it while landing=False) and nothing would
                    # resend. Repeats are ignored once disarmed.
                    self.landed_pub.publish(Bool(data=True))
            else:
                # Eased lift ramp. At a constant rate the velocity reference
                # jumps 0 -> lift_ramp_vel the cycle TAKEOFF lands and back to 0
                # at the top; _lift_vel is fed forward directly, so both ends
                # were velocity steps the drones absorbed as the aggressive
                # liftoff. Ease in over LIFT_SOFT_S, out over LIFT_SOFT_D metres.
                total = self.target_z - self.lift_z0
                self._lift_t += 1.0 / PLANNER_HZ
                ease_in = 0.5 * (1.0 - np.cos(
                    np.pi * min(self._lift_t / max(LIFT_SOFT_S, 1e-6), 1.0)))
                remaining = max(total - self.lift_progress, 0.0)
                ease_out = 0.5 * (1.0 - np.cos(
                    np.pi * min(remaining / max(LIFT_SOFT_D, 1e-6), 1.0)))
                # floor the shape so the climb always finishes: a pure ease-out
                # approaches the target asymptotically and lift_done never fires.
                shape = max(min(ease_in, ease_out), LIFT_SOFT_MIN)
                self.lift_progress = min(
                    total, self.lift_progress + shape * self.lift_ramp_vel / PLANNER_HZ)
                # Lateral trajectory runs once the lift tops out: the coupled OCP
                # tracks the moving load reference via _yref_at. When it closes,
                # latch the descent.
                lift_done = (self.lift_progress
                             >= (self.target_z - self.lift_z0) - 1e-6)
                if self.load_traj != 'hover' and lift_done:
                    self.traj_t += 1.0 / PLANNER_HZ
                    if self.traj.complete(self.traj_t):
                        self.descending = True
                        self.get_logger().info(
                            '[planner] load trajectory complete — descending')
        # signed vertical velocity of the lift target for the FF (from the actual
        # change this cycle): +ascend, -descend, 0 hold.
        self._lift_vel = (self.lift_progress - prev_progress) * PLANNER_HZ

        # Solve the planner OCP against the per-node lift/trajectory reference and
        # publish the horizon. Reseed (hard reconverge from x_init) only when there
        # is no valid warm start -- the very first solve, or after a failed one;
        # otherwise warm-start from last_X. On a ground start the solver was kept
        # warm on the live config all through creep (_prime_solver), so last_X is
        # already populated here and this first post-handover solve is a warm
        # refinement, not a cold reconverge.
        self.refs.update(self.hover_xy, self.lift_z0, self.lift_progress,
                         self.target_z, self._lift_vel, self.traj_t)
        drone_slot_pos = [self._drone_at(i) for i in range(self.n)]
        x_init = self.solver.build_x_init(self.load_state, drone_slot_pos)
        X, status = self.solver.solve_horizon(
            self.refs.yref_at, self.refs.q_ref_at, x_init,
            reseed=self.solver.last_X is None or self.solver.recover)
        if X is None:
            # Don't publish a degenerate solution and don't let it warm-start the
            # next cycle — drop the warm start and hold the last good reference.
            self.solver.recover = True
            self.solver.last_X = None
            self.get_logger().warn(
                f'[planner] solve status {status} — holding last reference, '
                f'will reconverge from x_init next cycle')
            return
        self.solver.recover = False
        self.solver.last_X = X
        self._publish_refs(X)

    def _prime_solver(self):
        """Keep the OCP warm during the creep phase so the creep->planner switch has
        a valid warm start. Solves a HOLD reference (current load pose, nominal taut
        hover) with node 0 pinned to the live measured state, but does NOT publish
        the horizon — the trackers stay on the creep refs. By handover last_X is a
        converged solution matching the current geometry, so the first planner cycle
        is a warm refinement rather than a cold reconverge (which showed up as a jump
        and a scrambled MPC path for a moment right after handover)."""
        drone_slot_pos = [self._drone_at(i) for i in range(self.n)]
        x_init = self.solver.build_x_init(self.load_state, drone_slot_pos)
        hold = self.refs.hold_yref(self.load_state)
        X, _status = self.solver.solve_horizon(
            lambda _k: hold, self.refs.q_ref_at, x_init,
            reseed=self.solver.last_X is None)
        self.solver.last_X = X          # None on a failed solve -> next tick reseeds

    def _publish_load_desired(self, target=None):
        """Publish the desired LOAD position [x, y, z]: the captured hover xy and
        the ramped lift target. Before handover (lift_z0 unset) the load isn't
        being lifted, so the desired height is just its current height.

        `target` overrides the computed desired position, for a subclass whose flight
        phase owns a different one (the dissipative network's p_des) -- otherwise the
        published desired silently disagrees with what is actually being commanded,
        which is exactly what the plot_run.py error analysis reads."""
        if self.hover_xy is None:
            return
        if target is not None:
            x0, y0, z_des = (float(target[0]), float(target[1]), float(target[2]))
        else:
            if self.lift_z0 is not None:
                z_des = min(self.target_z, self.lift_z0 + self.lift_progress)
            else:
                z_des = float(self.load_state[2])
            dx, dy, _, _ = self.traj.offset_at(self.traj_t)
            x0 = float(self.hover_xy[0] + dx)
            y0 = float(self.hover_xy[1] + dy)
        msg = Float64MultiArray()
        msg.data = [x0, y0, float(z_des)]
        self.load_ref_pub.publish(msg)

        # Horizon path for RViz: where the load is heading over the next N+1 nodes.
        #
        # ANCHORED ON THE MEASURED LOAD, like each drone's /drone_N/mpc_plan (whose
        # node 0 is the pinned measurement), so the path visibly emanates from the
        # payload instead of floating at the desired point. The tracking error is NOT
        # lost by this -- it is the gap between /payload/desired_position and the
        # measured load, which is what plot_run.py overlays.
        #
        # The lateral shape comes from evaluating the TRAJECTORY at traj_t + dt*k, as
        # reference_builder.yref_at does. It used to extrapolate along the current
        # velocity, which draws a straight tangent line -- so a circle rendered as a
        # line shooting off the path (0.56 m off it by the end of a 2 s horizon at
        # traj_speed 0.4, radius 0.5).
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        z_cap = self.target_z if self.lift_z0 is not None else z_des
        p_now = self.load_state[0:3]
        dx0, dy0, _, _ = self.traj.offset_at(self.traj_t)
        for k in range(self.N + 1):
            kx, ky, _, _ = self.traj.offset_at(self.traj_t + self.dt * k)
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(p_now[0]) + (kx - dx0)
            ps.pose.position.y = float(p_now[1]) + (ky - dy0)
            # z still shows the commanded climb/descent, capped at the target, but
            # measured from where the load actually is.
            ps.pose.position.z = min(z_cap,
                                     float(p_now[2]) + self._lift_vel * self.dt * k)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.load_plan_pub.publish(path)

    def prebuild_solvers(self, sizes):
        """Compile/load a planner OCP for each fleet size we might hand back to.

        Only n sets the OCP dimensions (attachment geometry is a runtime parameter --
        see LoadCableDynamics), so one solver per size covers any layout. They are
        built at STARTUP because an acados build is 9-40 s and cannot happen in
        flight; with a warm cache each is ~0.1 s. `tools/prebuild_planner.py` warms
        that cache after a model change.
        """
        for m in sorted({int(v) for v in sizes} | {self.n}):
            if m in self._solvers or m < 2:
                continue
            rho = attach_points(m, self.attach_radius, self.attach_z)
            dyn = LoadCableDynamics(m, self.load_mass, LOAD_INERTIA,
                                    [self.cable_len] * m, rho, DRONE_MASS)
            self._solvers[m] = (dyn, PlannerSolver(dyn, self.get_logger()), rho)
            self.get_logger().info(f'[planner] OCP ready for n={m}')

    def resize_fleet(self, new_n, drone_ids, rho=None):
        """Re-point the planner at a fleet of `new_n` drones, listed in slot order.

        Swaps the OCP (and its dynamics, ring and reference builder) for the
        pre-built one of that size, and rebuilds the slot->drone map from the
        surviving physical ids. The solver's warm start is dropped: the previous
        solution describes a different fleet, and reusing it would seed the first
        solve of the new size with a state vector of the wrong meaning.

        `rho` IS THE IMPORTANT ARGUMENT. The pre-built solver for new_n carries a
        nominal EVEN ring, and after a detach that is physically wrong: the cables
        that remain are still bolted to their original attach points, so three
        survivors of a four-ring sit at 0/90/180 deg, not at 0/120/240. Handing the
        OCP an even 3-gon makes it solve for a payload whose cables are somewhere
        they are not, and the moment balance it computes tips the load over -- seen
        on R0093, payload tilt 62 deg about 9 s after the hand-back. Pass the
        surviving subset of the ORIGINAL ring instead. This is only expressible
        because attachment geometry is a runtime parameter (LoadCableDynamics).

        Returns False (and changes nothing) if no solver was pre-built for new_n.
        """
        entry = self._solvers.get(int(new_n))
        if entry is None:
            self.get_logger().error(
                f'[planner] cannot resize to n={new_n}: no OCP built for that size. '
                f'Add it to handback_sizes.')
            return False
        if len(drone_ids) != int(new_n):
            self.get_logger().error(
                f'[planner] resize to n={new_n} got {len(drone_ids)} drone ids')
            return False
        self.dyn, self.solver, self.rho = entry
        self.n = int(new_n)
        if rho is not None:
            self.rho = [np.asarray(r, float).reshape(3) for r in rho]
            self.solver.set_geometry(rho=self.rho)
        else:
            self.solver.set_geometry(rho=self.rho)
        self._s_nom = nominal_cable_dirs(self.rho, 45.0)
        self.refs = ReferenceBuilder(self.dyn, self.n, self._s_nom, self.dt,
                                     self.traj)
        self.slot2drone = [int(d) for d in drone_ids]
        self.solver.last_X = None            # different fleet: no valid warm start
        self.N = self.solver.N
        self.dt = self.solver.dt
        self.get_logger().warn(
            f'[planner] FLEET RESIZED to n={self.n}; slot->drone {self.slot2drone}; '
            f'attach azimuths '
            f'{[round(float(np.degrees(np.arctan2(r[1], r[0]))), 1) for r in self.rho]} deg')
        return True

    def _enter_planner_phase(self, reason):
        """Transition creep -> coupled planner: latch the lift-ramp start height.
        No hard reconverge is forced here: the solver was kept warm on the live
        config through creep (_prime_solver), so the first planner solve warm-starts
        from that. For a start_taut air-start (no creep) last_X is still None at this
        point, so that first solve reseeds anyway."""
        self.phase = 'planner'
        self.lift_z0 = float(self.load_state[2])   # ramp lift from here
        self.lift_progress = 0.0
        self._lift_t = 0.0                         # restart the lift easing
        self._ff_t = 0.0                           # restart the cable-FF soft-start
        self._settle_left = self.handover_settle_s
        self.get_logger().info(
            f'[planner] {reason} — coupled planner active '
            f'(lift from z={self.lift_z0:.2f})')

    def _publish_ref(self, i, nodes):
        """Publish one drone's reference trajectory. nodes is a sequence of
        (p, v, a, a_cable) with three components each, one entry per horizon
        node. Single definition of the wire format described in the module
        docstring, so the four callers cannot disagree on field order."""
        data = [float(self.N + 1), float(self.dt)]
        for p, v, a, ac in nodes:
            data += [float(p[0]), float(p[1]), float(p[2]),
                     float(v[0]), float(v[1]), float(v[2]),
                     float(a[0]), float(a[1]), float(a[2]),
                     float(ac[0]), float(ac[1]), float(ac[2])]
        msg = Float64MultiArray()
        msg.data = data
        self.ref_pub[i].publish(msg)

    def _cable_taut_gate(self, i):
        """(gate, dist): `gate` in [0, 1] scales the cable-tension feedforward we
        publish to the tracker by how LENGTH-taut the cable measures right now —
        0 while the cable is slack, 1 once straightened (the slack->taut
        transition). Rigid cables are taut from spawn, so this is ~1 immediately;
        soft cables ramp it in as they straighten."""
        ls = self.load_state
        R = quat_to_rot_np(ls[3:7])
        attach = ls[0:3] + R @ self.rho[i]
        dist = float(np.linalg.norm(attach - self._drone_at(i)))
        d_lo = CABLE_TAUT_LO_FRAC * self.cable_len
        d_hi = CABLE_TAUT_HI_FRAC * self.cable_len
        len_gate = float(np.clip((dist - d_lo) / max(d_hi - d_lo, 1e-6), 0.0, 1.0))
        return len_gate, dist

    def _publish_refs(self, X):
        # Cable-FF soft-start: 0 through the grounded handover settle, easing to 1
        # over FF_EASE_S once the lift starts (clock advanced in _plan). Multiplied
        # into the published tension FF so it is never stepped onto the still-
        # grounded load -- that step made the drones lurch and scrambled the horizon.
        ff = float(np.clip(self._ff_t / FF_EASE_S, 0.0, 1.0))
        diag = []
        for i in range(self.n):
            gate, dist = self._cable_taut_gate(i)
            t_i = float(X[LOAD_DIM + CABLE_DIM * i + 12, 0])   # planned tension, node 0
            diag.append((i, dist, gate, t_i))
            nodes = []
            for k in range(self.N + 1):
                pos, vel, acc, cable = self.solver.drone_kinematics(X[:, k], i)
                nodes.append((pos, vel, acc, gate * ff * cable))
            # slot i's planned trajectory belongs to the physical drone occupying it
            self._publish_ref(self.slot2drone[i], nodes)

        # ~1 Hz: per-drone cable tautness so you can see when (and whether) the
        # cable term engages. dist -> CABLE_LEN means taut; gate is the applied
        # tension scale; t is the OCP's planned tension. If the drones fall while
        # gate stays 0, the cable never tautens before they lose it.
        self._diag_ctr = getattr(self, '_diag_ctr', 0) + 1
        if self._diag_ctr % int(max(PLANNER_HZ, 1)) == 0:
            s = '  '.join(f"d{i}:dist={d:.2f} gate={g:.2f} t={t:.2f}"
                          for (i, d, g, t) in diag)
            z_tgt = min(self.target_z, self.lift_z0 + self.lift_progress)
            # load tilt (angle of the load body-z off world-z) and each MEASURED
            # cable elevation, so an asymmetric divergence is visible before it
            # blows up: a rotating load / flattening cables drives the tension split.
            ls = self.load_state
            R = quat_to_rot_np(ls[3:7])
            tilt = np.degrees(np.arccos(np.clip(R[2, 2], -1.0, 1.0)))
            elevs = []
            for i in range(self.n):
                d = (ls[0:3] + R @ self.rho[i]) - self._drone_at(i)
                nd = np.linalg.norm(d)
                elevs.append(np.degrees(np.arcsin(np.clip(-d[2] / max(nd, 1e-6), -1, 1))))
            e = ' '.join(f"{x:.0f}" for x in elevs)
            self.get_logger().info(
                f"[planner cable] L={self.cable_len:.2f} load_z={self.load_state[2]:.2f} "
                f"z_tgt={z_tgt:.2f} ff={ff:.2f} tilt={tilt:.1f}deg elev=[{e}]  {s}")


def main(args=None):
    rclpy.init(args=args)
    node = LoadPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
