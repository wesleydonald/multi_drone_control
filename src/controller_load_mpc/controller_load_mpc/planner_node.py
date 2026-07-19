"""
planner_node.py
---------------
Centralized cable-suspended load planner node.

Runs the load-cable OCP (planner_ocp.py) at PLANNER_HZ. Each cycle it:
  1. builds x_init from mocap (NO EKF — sim ground truth):
       - load pose/twist from /payload/motion_capture_state
       - cable directions s_i = (p + R(q) rho_i - p_i)/||.|| from drone positions
       - cable rates / tensions / higher derivatives resampled from the previous
         OCP solution (nominal on the first solve)
  2. sets the load-pose reference (hover at captured x,y and TARGET_Z for now)
  3. solves (SQP-RTI), extracts each drone's reference trajectory via the
     kinematic constraint p_i = p + R(q) rho_i - l_i s_i (+ velocity)
  4. publishes /drone_{i}/reference_trajectory (Float64MultiArray):
       [n_nodes, dt,
        px0,py0,pz0, vx0,vy0,vz0, ax0,ay0,az0, cx0,cy0,cz0, px1,...]  (world frame)
       -> 12 fields per node.
     where a_i is the required SPECIFIC thrust acceleration (f_i / m_i) — its
     magnitude maps to the tracker throttle and its direction to the tracker's
     desired (yaw-free) attitude (feedforward, Sun et al. piece 1) — and
     c_i = t_i s_i / m_i is the cable tension ACCELERATION on the drone, which the
     cable-aware tracker (controller_quad_load) adds to its prediction model so its
     feedback no longer fights the cable. The cable-blind tracker
     (controller_mpc_multi) simply ignores the extra c_i fields.

These references are consumed by the per-drone tracker (controller_mpc_multi),
which replaces its hardcoded circle with this stream (tracker hookup = next step).

Geometry MUST match three_soft.sdf (see generate_soft_world.py defaults).
"""
import numpy as np
import casadi as ca
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String, Bool
from interfaces.msg import MotionCaptureState

from .load_cable_dynamics import LoadCableDynamics, LOAD_DIM, CABLE_DIM
from .planner_ocp import generate_load_ocp, nominal_hover_state

# ── world geometry / params (match three_soft.sdf) ─────────────────────────
N_DRONES      = 3
LOAD_MASS     = 0.4
LOAD_INERTIA  = [1.67e-3, 1.67e-3, 3.33e-3]
CABLE_LEN     = 0.6            # slack 0.6 m cables (see generate_soft_world --cable-len)
ATTACH_RADIUS = 0.08
ATTACH_Z      = 0.025          # attach height above load CoG (load frame)
DRONE_MASS    = 0.6
PLANNER_HZ    = 10.0
STEADY_ITERS  = 5             # SQP iterations per planner cycle once running (was 1;
                              # 1 RTI step can't track the taut transition online)
CREEP_VEL     = 0.10          # m/s straight-up rise during the slack takeoff phase
CREEP_LEAD    = 0.10          # constant z lead held while grounded, to initiate climb
LIFTOFF_MARGIN = 0.05         # drone must rise this far above spawn to start the ramp
TAUT_SWITCH_GATE = 0.95       # hand over from creep to the coupled planner once
                              # every cable is at least this taut
TARGET_Z      = 0.6           # load hover height reference
LIFT_RAMP_VEL = 0.05          # m/s: fixed-schedule load lift rate after handover
                              # (decoupled from measured load_z, see _yref)
LAND_DRONE_Z  = 0.20          # LAND stops once every drone is at/below this z (m)
LAND_MAX_DROP = 1.50          # m: safety floor on how far below the handover
                              # height the LAND descent pushes the reference
LAND_VEL      = 0.20          # m/s: LAND descent rate (own knob, faster than the
                              # slow lift_ramp_vel used for a gentle takeoff)
LIFT_STEP     = 0.04          # max commanded climb above current load z. Kept
                              # SMALL: a large lead makes the drones build climb
                              # speed and overshoot the taut transition, spiking
                              # cable tension beyond the drones' thrust authority
                              # (they get physically overpowered and fall). Small
                              # step -> slow approach to taut -> bounded tension.
# Cable tautness gate: the planner OCP assumes ideal taut cables, but the Gazebo
# cables spawn SLACK, so feeding the predicted tension while the cable is loose
# makes the tracker over-estimate the pull and lurch at takeoff. We scale the
# published cable acceleration by how taut the cable physically is RIGHT NOW,
# measured as the drone->attach distance vs cable length: 0 while clearly slack,
# ramping to 1 as the cable straightens. Tune CABLE_TAUT_LO_FRAC up toward 1.0 to
# delay tension feed-in (only very near full extension), down to feed in earlier.
CABLE_TAUT_LO_FRAC = 0.85     # start ramping tension in at 85% of cable length
CABLE_TAUT_HI_FRAC = 1.00     # full tension once the cable is fully extended
CABLE_ENGAGE_HEIGHT = 0.15    # load must lift this far off its spawn/ground height
                              # before the FULL cable-tension feedforward engages.
                              # A geometrically-taut cable to a GROUNDED load bears
                              # ~0 tension; feeding a_cable then destabilises the
                              # tracker (it tilts out to fight a phantom pull).


def quat_to_rot_np(q):
    """Rotation matrix from quaternion q = [w, x, y, z] (numpy)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)],
        [2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)],
    ])


class LoadPlanner(Node):
    def __init__(self):
        super().__init__('load_planner')

        # ── geometry / mode params (override the module defaults per world) ──
        # three_soft.sdf spawns SLACK 0.6 m cables (creep to take up slack);
        # three_soft_paper.sdf spawns TAUT 1.0 m cables at ~45deg (start_taut).
        self.cable_len = float(
            self.declare_parameter('cable_len', CABLE_LEN).value)
        self.attach_radius = float(
            self.declare_parameter('attach_radius', ATTACH_RADIUS).value)
        self.attach_z = float(
            self.declare_parameter('attach_z', ATTACH_Z).value)
        # must match the payload mass in the world SDF - the planner sizes the
        # cable tension off this, so a mismatch scales every drone's tension FF.
        # three_soft.sdf/three_rigid_short.sdf = 0.4, three_soft_paper.sdf = 0.1.
        self.load_mass = float(
            self.declare_parameter('load_mass', LOAD_MASS).value)
        # start_taut: cables already taut at spawn (elevated world) -> skip the
        # creep takeoff and hand straight to the coupled planner.
        self.start_taut = bool(
            self.declare_parameter('start_taut', False).value)
        # lift schedule (params so a HOLD test = lift_ramp_vel:=0.0): the load
        # target rises from the handover height at lift_ramp_vel up to target_z.
        self.target_z = float(
            self.declare_parameter('target_z', TARGET_Z).value)
        self.lift_ramp_vel = float(
            self.declare_parameter('lift_ramp_vel', LIFT_RAMP_VEL).value)
        # planner_mode:
        #   'coupled'   — solve the full load-cable OCP online each cycle, pinning
        #                 node 0 to the measured state (re-closes the loop on mocap).
        #   'kinematic' — OPEN-LOOP feedforward: lift the latched taut config rigidly
        #                 and stream the analytic loaded-hover FF. No online OCP, no
        #                 mocap feedback -> the tracker provides the restoring force
        #                 (the coupled loop was unstable: it overshot & diverged).
        self.planner_mode = str(
            self.declare_parameter('planner_mode', 'coupled').value)
        # ff_gate_mode: how the cable/thrust feedforward is gated in.
        #   'airborne' — ramp FF in as the LOAD lifts off the ground. Needed for
        #                SOFT cables (feeding full tension while the position-
        #                dependent soft cable is slack destabilises the tracker).
        #   'taut'     — engage full FF as soon as the cable is geometrically taut
        #                (=1 from spawn for RIGID cables). The drones then lift the
        #                load from the start, so they don't slide inward or run
        #                ahead of the lift ramp. Use for rigid (paper) worlds.
        self.ff_gate_mode = str(
            self.declare_parameter('ff_gate_mode', 'airborne').value)
        # ── LOAD reference trajectory (kinematic mode) ──────────────────────
        # After the vertical lift tops out at target_z, translate the whole taut
        # formation laterally so the LOAD follows a trajectory. Because kinematic
        # mode rigidly translates the latched taut config, a pure translation
        # keeps the cable geometry (so the cached FF stays valid). Shapes:
        #   'hover'  — no lateral motion (current behaviour: lift then hold).
        #   'line_x' — move +x at traj_speed for traj_distance metres, then hold.
        #   'circle' — horizontal circle of traj_radius at tangential traj_speed.
        # Keep traj_speed slow: centripetal/lateral accel is NOT fed forward, so
        # fast motion would need cable tilt this open-loop translation can't model.
        self.load_traj = str(
            self.declare_parameter('load_traj', 'hover').value)
        self.traj_speed = float(
            self.declare_parameter('traj_speed', 0.1).value)      # m/s lateral
        self.traj_distance = float(
            self.declare_parameter('traj_distance', 1.0).value)   # m (line_x)
        self.traj_radius = float(
            self.declare_parameter('traj_radius', 0.5).value)     # m (circle)
        self.traj_t = 0.0            # elapsed lateral-trajectory time (post-hover)
        self.descending = False      # latched once the lateral trajectory closes
        self._land_to_ground = False # LAND command: descend all the way to ground
        self._landed = False         # descent finished, load back at start height
        self._lift_vel = 0.0         # signed vertical velocity of the lift target
        # LAND stops once every drone is at/below this height (override per world).
        self.land_drone_z = float(
            self.declare_parameter('land_drone_z', LAND_DRONE_Z).value)
        # LAND descent rate (separate from lift_ramp_vel so a slow takeoff doesn't
        # force a slow landing).
        self.land_vel = float(self.declare_parameter('land_vel', LAND_VEL).value)
        self.get_logger().info(
            f'[planner] geometry: cable_len={self.cable_len:.3f} '
            f'attach_radius={self.attach_radius:.3f} attach_z={self.attach_z:.3f} '
            f'load_mass={self.load_mass:.3f} '
            f'start_taut={self.start_taut} target_z={self.target_z:.3f} '
            f'lift_ramp_vel={self.lift_ramp_vel:.3f} mode={self.planner_mode} '
            f'ff_gate_mode={self.ff_gate_mode} load_traj={self.load_traj} '
            f'traj_speed={self.traj_speed:.3f} traj_distance={self.traj_distance:.3f} '
            f'traj_radius={self.traj_radius:.3f}')

        self.n = N_DRONES
        self.rho = [np.array([self.attach_radius * np.cos(2 * np.pi * k / self.n),
                              self.attach_radius * np.sin(2 * np.pi * k / self.n),
                              self.attach_z]) for k in range(self.n)]

        self.dyn = LoadCableDynamics(
            self.n, self.load_mass, LOAD_INERTIA, [self.cable_len] * self.n,
            self.rho, DRONE_MASS)
        self.get_logger().info(
            f'[planner] building OCP (nx={self.dyn.nx}, nu={self.dyn.nu})...')
        self.ocp, self.solver = generate_load_ocp(self.dyn)
        self.N = self.ocp.solver_options.N_horizon
        self.dt = self.ocp.solver_options.tf / self.N

        # numeric kinematic functions for reference extraction
        self.pos_fun = [ca.Function(f'p{i}', [self.dyn.x], [self.dyn.quad_position(i)])
                        for i in range(self.n)]
        self.vel_fun = [ca.Function(f'v{i}', [self.dyn.x], [self.dyn.quad_velocity(i)])
                        for i in range(self.n)]
        # required specific thrust acceleration a_i = f_i / m_i (feedforward)
        self.acc_fun = [ca.Function(f'a{i}', [self.dyn.x],
                                    [self.dyn.thrust_vec(i) / self.dyn.mi[i]])
                        for i in range(self.n)]
        # cable tension acceleration a_cable_i = t_i s_i / m_i (world frame) — the
        # known external pull the cable-aware tracker adds to its drone model
        self.cable_fun = [ca.Function(f'ac{i}', [self.dyn.x],
                                      [self.dyn.cable_accel(i)])
                          for i in range(self.n)]

        # state
        self.load_state = None                 # [p(3), q(4 wxyz), v(3), w(3)]
        self.drone_pos = [None] * self.n
        self.last_X = None                     # previous solution (nx, N+1)
        self.hover_xy = None                   # captured load x,y for the reference
        self.first_solve = True
        self._recover = False                  # reseed+reconverge after a failed solve
        self.phase = 'creep'                   # 'creep' (slow rise to taut) -> 'planner'
        self.creep_anchor = None               # latched spawn xy/z per drone
        self.creep_climb = 0.0                 # accumulated creep climb height (m)
        self.lifted_off = False                # gate the creep climb on real liftoff
        self.takeoff_seen = False              # gate the lift ramp on TAKEOFF (see below)
        self.lift_z0 = None                    # load height latched at handover
        self.lift_progress = 0.0               # ramped lift above lift_z0 (m)

        # subs
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_cb, 5)
        # /fleet/step is published continuously (0 before TAKEOFF, then it starts
        # incrementing once the fleet is flying). A step > 0 is our cue that
        # TAKEOFF has happened and it's safe to begin the lift ramp (see _plan).
        self.create_subscription(
            Int32, '/fleet/step',
            lambda msg: setattr(self, 'takeoff_seen',
                                self.takeoff_seen or msg.data > 0), 5)
        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda msg, k=i: self._drone_cb(msg, k), 5)
        # /fleet/command (ARM|TAKEOFF|DISARM|ESTOP|LAND). We only act on LAND:
        # start the descent (stop the lateral trajectory, lower the load back to
        # the handover height). The central controller disarms once we finish.
        self.create_subscription(
            String, '/fleet/command', self._fleet_command_cb, 10)

        # pubs
        self.ref_pub = [self.create_publisher(
            Float64MultiArray, f'/drone_{i}/reference_trajectory', 5)
            for i in range(self.n)]
        # published True once the LAND descent has finished (load back at the
        # handover height), so the central controller knows it can disarm.
        self.landed_pub = self.create_publisher(Bool, '/fleet/landed', 1)
        # desired LOAD position [x, y, z] — the payload setpoint the planner is
        # driving toward (captured hover xy + ramped lift target). Logged by the
        # drone-0 tracker so plot_run.py can overlay payload desired vs actual.
        self.load_ref_pub = self.create_publisher(
            Float64MultiArray, '/payload/desired_position', 5)

        self.create_timer(1.0 / PLANNER_HZ, self._plan)
        self.get_logger().info('[planner] ready, waiting for mocap...')

    # ── mocap callbacks ─────────────────────────────────────────────────────
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
        self.drone_pos[i] = np.array([p.x, p.y, p.z])

    def _fleet_command_cb(self, msg: String):
        cmd = msg.data.strip().upper()
        if cmd != 'LAND':
            return
        if self.phase != 'planner' or not self.takeoff_seen:
            self.get_logger().warn('[planner] LAND ignored - not flying yet')
            return
        if self.descending:
            return                       # already landing
        # Stop the lateral trajectory (traj_t stops advancing while descending)
        # and ramp the reference down to the handover height. _land_to_ground
        # marks this as a commanded LAND so we publish /fleet/landed at the end
        # (the central controller then disarms and the drones settle down).
        self.descending = True
        self._land_to_ground = True
        self._landed = False
        self.get_logger().info('[planner] LAND - descending to the floor')

    # ── x_init assembly ─────────────────────────────────────────────────────
    def _build_x_init(self):
        ls = self.load_state
        p, q = ls[0:3], ls[3:7]
        R = quat_to_rot_np(q)
        # base: resample previous solution (advance one node) or nominal hover
        if self.last_X is not None:
            x = self.last_X[:, 1].copy()
        else:
            x = nominal_hover_state(self.dyn, load_pos=tuple(p))
        # overwrite load block with fresh mocap
        x[0:3] = p
        x[3:6] = ls[7:10]      # v
        x[6:10] = q            # q (wxyz)
        x[10:13] = ls[10:13]   # w
        # overwrite each cable direction s_i from geometry (keep rates/tension)
        for i in range(self.n):
            attach = p + R @ self.rho[i]
            d = attach - self.drone_pos[i]
            nrm = np.linalg.norm(d)
            if nrm > 1e-6:
                b = LOAD_DIM + CABLE_DIM * i
                x[b:b + 3] = d / nrm
        return x

    def _yref(self):
        x0, y0 = self.hover_xy
        t_nom = self.dyn.m * 9.81 / (self.n * np.sin(np.deg2rad(45.0)))
        # Time-based lift ramp, decoupled from the MEASURED load height. The old
        # self-pacing target (load_z + LIFT_STEP) was positive feedback: a load dip
        # lowered the target, which cut thrust, which dropped the load further — a
        # monotonic collapse into the ground. Here the target rises on a fixed slow
        # schedule from the handover height, so the load tracks a clean setpoint and
        # the loop is no longer self-reinforcing.
        z_target = min(self.target_z, self.lift_z0 + self.lift_progress)
        pose = [x0, y0, z_target, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # p, v, e_att, w
        y = np.array(pose + [t_nom] * self.n
                     + [0.0] * (3 * self.n)        # r_vec ref (no cable swing)
                     + [0.0] * self.dyn.nu)
        return y, y[:-self.dyn.nu]

    # ── plan step ───────────────────────────────────────────────────────────
    def _plan(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return

        self._publish_load_desired()

        # ── Phase 1: creep takeoff ───────────────────────────────────────────
        # The coupled OCP assumes taut cables. While the cables are slack the
        # drones must climb ~0.4 m to take up the slack; running the lift planner
        # through that makes them overshoot and snap the soft cable taut, which
        # the open-loop tracker cannot ride. Instead we bypass the planner and
        # creep each drone straight up slowly until every cable is taut, then hand
        # over. This keeps the slack->taut transition gentle (low velocity at
        # contact) so tension never spikes past the drones' thrust authority.
        # start_taut worlds spawn with taut cables (drones elevated) — skip the
        # creep takeoff entirely and enter the coupled planner on the first cycle.
        if self.phase == 'creep' and self.start_taut:
            self._enter_planner_phase('start_taut')

        gates = [self._cable_taut_gate(i) for i in range(self.n)]
        if self.phase == 'creep':
            self._publish_creep_refs(gates)
            if all(g >= TAUT_SWITCH_GATE for (g, _d) in gates):
                self._enter_planner_phase('cables taut')
            return

        # Don't advance the lift ramp until TAKEOFF. The planner runs continuously
        # from the moment it gets mocap, but the drones sit idle until the fleet
        # manager broadcasts /fleet/step (published only after TAKEOFF). Ramping
        # before then runs lift_progress away (metres above the still-idle drones),
        # so the reference yanks them up and they overshoot the instant they arm.
        # (A position-based liftoff test can't be used here: in the elevated
        # start_taut world the drones spawn airborne and the held reference keeps
        # them at spawn, so they never "rise above spawn" to trip it.)
        # Vertical phases: ASCEND to target_z -> run the lateral trajectory ->
        # DESCEND back to the handover height once the trajectory closes.
        prev_progress = self.lift_progress
        if self.takeoff_seen:
            if self.descending:
                # Descent. Auto-descent (trajectory finished) stops at the
                # handover height (lift_progress -> 0). A LAND command keeps
                # driving the reference down below that (lift_progress negative)
                # until every drone is on the floor: the reference goes under the
                # ground, but the drones just stop once they physically hit it.
                # NOTE: this tracks cleanly only when the drones follow the
                # reference closely; if thrust_ratio is mistuned the load floats
                # ~1 m above its reference and the descent goes unstable.
                # LAND descends at land_vel; auto-descent keeps lift_ramp_vel.
                floor = -LAND_MAX_DROP if self._land_to_ground else 0.0
                rate = self.land_vel if self._land_to_ground else self.lift_ramp_vel
                self.lift_progress = max(
                    floor, self.lift_progress - rate / PLANNER_HZ)
                if self._land_to_ground:
                    have = all(d is not None for d in self.drone_pos)
                    max_dz = max((d[2] for d in self.drone_pos if d is not None),
                                 default=1e9)
                    done = (have and max_dz <= self.land_drone_z) \
                        or self.lift_progress <= floor + 1e-6
                else:
                    done = self.lift_progress <= 1e-6
                if done and not self._landed:
                    self._landed = True
                    if self._land_to_ground:
                        self.landed_pub.publish(Bool(data=True))
                    self.get_logger().info(
                        '[planner] descent complete — drones on the floor')
            else:
                self.lift_progress = min(
                    self.target_z - self.lift_z0,
                    self.lift_progress + self.lift_ramp_vel / PLANNER_HZ)
                # Lateral trajectory runs once hovering (kinematic mode only); when
                # it closes, latch the descent.
                lift_done = (self.lift_progress
                             >= (self.target_z - self.lift_z0) - 1e-6)
                if (self.planner_mode == 'kinematic' and self.load_traj != 'hover'
                        and lift_done):
                    self.traj_t += 1.0 / PLANNER_HZ
                    if self._traj_complete():
                        self.descending = True
                        self.get_logger().info(
                            '[planner] load trajectory complete — descending')
        # signed vertical velocity of the lift target for the FF (from the actual
        # change this cycle): +ascend, -descend, 0 hold.
        self._lift_vel = (self.lift_progress - prev_progress) * PLANNER_HZ

        # Open-loop kinematic feedforward: no OCP, no mocap feedback loop.
        if self.planner_mode == 'kinematic':
            self._publish_kinematic_refs(self._lift_vel)
            return

        x_init = self._build_x_init()
        yref, yref_e = self._yref()
        q_ref = np.array([1.0, 0.0, 0.0, 0.0])

        for k in range(self.N):
            self.solver.set(k, 'yref', yref)
            self.solver.set(k, 'p', q_ref)
        self.solver.set(self.N, 'yref', yref_e)
        self.solver.set(self.N, 'p', q_ref)

        # Reseed all nodes from x_init on the first solve, OR when recovering from a
        # failed solve (the previous warm start is NaN/garbage and would poison this
        # one). Otherwise keep the warm start from last_X.
        if self.first_solve or self._recover:
            for k in range(self.N + 1):
                self.solver.set(k, 'x', x_init)
        self.solver.set(0, 'lbx', x_init)
        self.solver.set(0, 'ubx', x_init)

        # A single RTI iteration cannot track the stiff soft-cable transition online,
        # so the planner diverges and emits degenerate references. Take several SQP
        # iterations per cycle (more on cold start / recovery) to stay converged.
        iters = 15 if (self.first_solve or self._recover) else STEADY_ITERS
        status = 0
        for _ in range(iters):
            status = self.solver.solve()
        self.first_solve = False

        if status != 0:
            # Don't publish a degenerate solution and don't let it warm-start the
            # next cycle — reseed from x_init next time and hold the last good ref.
            self._recover = True
            self.get_logger().warn(
                f'[planner] solve status {status} — holding last reference, '
                f'will reconverge from x_init next cycle')
            return
        self._recover = False

        X = np.array([self.solver.get(k, 'x') for k in range(self.N + 1)]).T
        self.last_X = X
        self._publish_refs(X)

    def _traj_complete(self):
        """True once the lateral load trajectory has finished, so the descent can
        begin. 'hover' never completes (holds indefinitely, the old behaviour)."""
        if self.load_traj == 'line_x':
            return self.traj_speed * self.traj_t >= self.traj_distance
        if self.load_traj == 'circle':
            w = self.traj_speed / max(self.traj_radius, 1e-6)
            return self.traj_t >= 2.0 * np.pi / w
        return False

    def _load_offset(self):
        """Lateral (x, y) offset + velocity of the LOAD reference at the current
        traj_t. The whole taut formation is translated by this, so the load
        follows it. Returns (dx, dy, vx, vy); zero until the lift has topped out."""
        t = self.traj_t
        if self.load_traj == 'line_x':
            dx = min(self.traj_speed * t, self.traj_distance)
            vx = self.traj_speed if dx < self.traj_distance else 0.0
            return dx, 0.0, vx, 0.0
        if self.load_traj == 'circle':
            r = max(self.traj_radius, 1e-6)
            w = self.traj_speed / r                    # angular rate
            period = 2.0 * np.pi / w                    # time for one full loop
            if t >= period:
                # completed one revolution -> back at the start point, hold there.
                return 0.0, 0.0, 0.0, 0.0
            dx = r * np.sin(w * t)                     # starts at (0,0), heads +x
            dy = r * (1.0 - np.cos(w * t))             # then curves +y
            vx = self.traj_speed * np.cos(w * t)
            vy = self.traj_speed * np.sin(w * t)
            return dx, dy, vx, vy
        return 0.0, 0.0, 0.0, 0.0

    def _publish_load_desired(self):
        """Publish the desired LOAD position [x, y, z]: the captured hover xy and
        the ramped lift target. Before handover (lift_z0 unset) the load isn't
        being lifted, so the desired height is just its current height."""
        if self.hover_xy is None:
            return
        if self.lift_z0 is not None:
            z_des = min(self.target_z, self.lift_z0 + self.lift_progress)
        else:
            z_des = float(self.load_state[2])
        dx, dy, _vx, _vy = self._load_offset()
        msg = Float64MultiArray()
        msg.data = [float(self.hover_xy[0] + dx), float(self.hover_xy[1] + dy),
                    float(z_des)]
        self.load_ref_pub.publish(msg)

    def _enter_planner_phase(self, reason):
        """Transition creep -> coupled planner: latch the lift-ramp start height
        and force a hard reconverge from x_init on the first solve."""
        self.phase = 'planner'
        self._recover = True              # hard-reconverge from x_init on entry
        self.lift_z0 = float(self.load_state[2])   # ramp lift from here
        self.lift_progress = 0.0
        # Latch the taut spawn config for the kinematic (open-loop) generator: the
        # per-drone cable direction, tension, and analytic FF, computed ONCE from
        # the geometry at handover. The whole rigid config is then translated up.
        self.kin_drone0 = [self.drone_pos[i].astype(float).copy()
                           for i in range(self.n)]
        self.kin_acable, self.kin_athrust = [], []
        g_vec = np.array([0.0, 0.0, self.dyn.g])
        for i in range(self.n):
            attach = self.load_state[0:3] + self.rho[i]        # level load
            d = attach - self.kin_drone0[i]
            s = d / max(np.linalg.norm(d), 1e-6)               # drone->load unit
            sin_elev = max(-s[2], 1e-3)                         # vertical share
            t_i = self.dyn.m * self.dyn.g / (self.n * sin_elev)
            a_cable = t_i * s / self.dyn.mi[i]                  # down-and-in
            self.kin_acable.append(a_cable)
            self.kin_athrust.append(g_vec - a_cable)           # up-and-out FF
        self.get_logger().info(
            f'[planner] {reason} — {self.planner_mode} planner active '
            f'(lift from z={self.lift_z0:.2f})')

    def _publish_kinematic_refs(self, lift_vel):
        """Open-loop feedforward reference: translate the latched taut config
        straight up by lift_progress and stream the analytic loaded-hover FF.

        This is an ABSOLUTE reference (independent of the live mocap), so the
        tracker sees a real position error when it drifts and pulls itself back —
        unlike the coupled planner, whose node-0 pinned to the measured state gave
        no restoring force and let the system overshoot and diverge. Because both
        load and every drone rise by the same lift_progress, the taut geometry
        (cable length, elevation, tension) is exactly preserved, so the cached FF
        stays valid throughout the lift."""
        # Lateral load-trajectory offset + velocity (same for every drone: the
        # whole taut formation translates together, preserving cable geometry).
        dx, dy, vx, vy = self._load_offset()
        gates = [self._cable_taut_gate(i) for i in range(self.n)]
        # unloaded level-hover FF: just support the drone's own weight, no tilt.
        a_unloaded = np.array([0.0, 0.0, self.dyn.g])
        for i in range(self.n):
            gate, _d = gates[i]
            base = self.kin_drone0[i]
            # Blend the WHOLE feedforward by the load-bearing gate, not just the
            # cable term: at gate 0 (load still resting on the ground) the drone
            # gets a plain unloaded level hover FF; at gate 1 (load fully on the
            # cables) it gets the loaded tilted+high-thrust FF. Leaving a_thrust at
            # the loaded value while the load is unloaded over-thrusts & over-tilts
            # the drone, driving it up-and-out (the "sliding away" failure).
            a_i = (1.0 - gate) * a_unloaded + gate * self.kin_athrust[i]
            ac_i = gate * self.kin_acable[i]
            data = [float(self.N + 1), float(self.dt)]
            for k in range(self.N + 1):
                x_off = dx + vx * self.dt * k
                y_off = dy + vy * self.dt * k
                z_off = self.lift_progress + lift_vel * self.dt * k
                p_i = [float(base[0] + x_off), float(base[1] + y_off),
                       float(base[2] + z_off)]
                v_i = [float(vx), float(vy), float(lift_vel)]
                data += [*p_i, *v_i, *a_i.tolist(), *ac_i.tolist()]
            msg = Float64MultiArray()
            msg.data = data
            self.ref_pub[i].publish(msg)

        self._diag_ctr = getattr(self, '_diag_ctr', 0) + 1
        if self._diag_ctr % int(max(PLANNER_HZ, 1)) == 0:
            z_tgt = min(self.target_z, self.lift_z0 + self.lift_progress)
            s = '  '.join(f"d{i}:g={g:.2f}" for i, (g, _d) in enumerate(gates))
            dx, dy, _vx, _vy = self._load_offset()
            phase = ('descend' if self.descending else
                     ('lift' if self._lift_vel > 1e-6 else 'traj/hold'))
            self.get_logger().info(
                f"[planner kinematic] load_z={self.load_state[2]:.2f} "
                f"z_tgt={z_tgt:.2f} lift={self.lift_progress:.2f} phase={phase} "
                f"traj={self.load_traj} off=({dx:+.2f},{dy:+.2f}) "
                f"|aT|={np.linalg.norm(self.kin_athrust[0]):.2f} "
                f"|aC|={np.linalg.norm(self.kin_acable[0]):.2f}  {s}")

    def _publish_creep_refs(self, gates):
        """Phase-1 takeoff: command each drone to rise straight up at CREEP_VEL,
        level attitude, no cable force.

        The xy reference is ANCHORED to each drone's takeoff position (not its
        live position) and the z reference is a time accumulator, so the reference
        is a fixed point in the air the drone must hold. This gives a real
        horizontal position-hold that resists the inward pull of the tautening
        cable — anchoring to the live position instead lets the drone drift inward
        and wind the attitude up until it tips over."""
        if self.creep_anchor is None:                 # latch spawn xy/z once
            self.creep_anchor = [self.drone_pos[i].astype(float).copy()
                                 for i in range(self.n)]
        # Don't accumulate the climb until the drones actually leave the ground.
        # The planner runs continuously, but the drones sit disarmed until TAKEOFF;
        # accumulating before then makes the reference run away (metres above the
        # grounded drones), so they rocket up and overshoot when finally armed.
        # While grounded, hold a small constant lead so takeoff still initiates.
        if not self.lifted_off:
            self.creep_climb = CREEP_LEAD
            if any(self.drone_pos[i][2] > self.creep_anchor[i][2] + LIFTOFF_MARGIN
                   for i in range(self.n)):
                self.lifted_off = True
        else:
            self.creep_climb += CREEP_VEL / PLANNER_HZ

        for i in range(self.n):
            ax, ay, az = self.creep_anchor[i]
            data = [float(self.N + 1), float(self.dt)]
            for k in range(self.N + 1):
                z = float(az + self.creep_climb + CREEP_VEL * self.dt * k)
                p_i = [float(ax), float(ay), z]
                v_i = [0.0, 0.0, CREEP_VEL]
                a_i = [0.0, 0.0, self.dyn.g]   # hover thrust, level -> no tilt
                ac_i = [0.0, 0.0, 0.0]          # slack: no cable force
                data += [*p_i, *v_i, *a_i, *ac_i]
            msg = Float64MultiArray()
            msg.data = data
            self.ref_pub[i].publish(msg)

        self._diag_ctr = getattr(self, '_diag_ctr', 0) + 1
        if self._diag_ctr % int(max(PLANNER_HZ, 1)) == 0:
            s = '  '.join(f"d{i}:dist={d:.2f} gate={g:.2f}"
                          for i, (g, d) in enumerate(gates))
            self.get_logger().info(f"[planner creep] vz={CREEP_VEL:.2f}  {s}")

    def _cable_taut_gate(self, i):
        """(gate, dist): `gate` in [0, 1] scales the cable-tension feedforward we
        publish to the tracker. It is the product of two factors:

          1. LENGTH taut: measured drone->attach distance vs cable length — 0 while
             the cable is slack, 1 once straightened (the slack->taut transition).
          2. LOAD airborne: how far the payload has lifted off its spawn/ground
             height. This is the important one — a cable can be geometrically taut
             (factor 1) while the load still RESTS ON THE GROUND, in which case the
             real tension is ~0. Feeding the full modelled a_cable then makes the
             tracker tilt outward to fight a pull that isn't there and it flies out
             (diagnosed via the HOLD test). So we suppress a_cable until the load is
             actually bearing on the cables, ramping it in over CABLE_ENGAGE_HEIGHT.
        """
        ls = self.load_state
        R = quat_to_rot_np(ls[3:7])
        attach = ls[0:3] + R @ self.rho[i]
        dist = float(np.linalg.norm(attach - self.drone_pos[i]))
        d_lo = CABLE_TAUT_LO_FRAC * self.cable_len
        d_hi = CABLE_TAUT_HI_FRAC * self.cable_len
        len_gate = float(np.clip((dist - d_lo) / max(d_hi - d_lo, 1e-6), 0.0, 1.0))
        if self.ff_gate_mode == 'taut':
            # rigid cable: taut from spawn, no soft-spring instability -> engage the
            # full FF immediately so the drones lift the load without sliding in.
            return len_gate, dist
        # 'airborne': also ramp by how far the LOAD has lifted off its ground height
        # (once we've latched it at handover) — needed for soft cables.
        if self.lift_z0 is not None:
            lifted = float(ls[2]) - self.lift_z0
            air_gate = float(np.clip(lifted / CABLE_ENGAGE_HEIGHT, 0.0, 1.0))
        else:
            air_gate = 1.0
        return len_gate * air_gate, dist

    def _publish_refs(self, X):
        diag = []
        for i in range(self.n):
            gate, dist = self._cable_taut_gate(i)
            t_i = float(X[LOAD_DIM + CABLE_DIM * i + 12, 0])   # planned tension, node 0
            diag.append((i, dist, gate, t_i))
            data = [float(self.N + 1), float(self.dt)]
            for k in range(self.N + 1):
                xk = X[:, k]
                p_i = np.array(self.pos_fun[i](xk)).flatten()
                v_i = np.array(self.vel_fun[i](xk)).flatten()
                a_i = np.array(self.acc_fun[i](xk)).flatten()
                ac_i = gate * np.array(self.cable_fun[i](xk)).flatten()
                data += [*p_i, *v_i, *a_i, *ac_i]
            msg = Float64MultiArray()
            msg.data = data
            self.ref_pub[i].publish(msg)

        # ~1 Hz: per-drone cable tautness so you can see when (and whether) the
        # cable term engages. dist -> CABLE_LEN means taut; gate is the applied
        # tension scale; t is the OCP's planned tension. If the drones fall while
        # gate stays 0, the cable never tautens before they lose it.
        self._diag_ctr = getattr(self, '_diag_ctr', 0) + 1
        if self._diag_ctr % int(max(PLANNER_HZ, 1)) == 0:
            s = '  '.join(f"d{i}:dist={d:.2f} gate={g:.2f} t={t:.2f}"
                          for (i, d, g, t) in diag)
            z_tgt = min(self.target_z, self.lift_z0 + self.lift_progress)
            self.get_logger().info(
                f"[planner cable] L={self.cable_len:.2f} load_z={self.load_state[2]:.2f} "
                f"z_tgt={z_tgt:.2f}  {s}")


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
