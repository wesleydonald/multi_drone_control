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

Two modes (planner_mode):
    coupled    solves the load-cable OCP (planner_ocp.py) online, pinning node 0
               to the measured state. This is the paper's method (Sun et al. 2025)
               and the default on the paper-implementation branch. x_init is built
               from mocap -- load pose/twist, cable directions s_i AND cable
               angular velocities r_i are all measured; only the unobservable
               higher cable states (rd_i, rdd_i) and tensions (t_i, td_i) are
               resampled from the previous solution (paper Fig 8). Requires a TAUT
               start: the paper assumes taut cables throughout and never lifts off
               the ground, so run it with start_taut and handover_elev_deg=0.
    kinematic  open-loop feedforward. Lifts the latched taut config rigidly and
               streams the analytic loaded-hover FF. The pre-paper working path,
               kept as a fallback.

Geometry must match the world SDF (see generate_rigid_world.py).
"""
import numpy as np
import casadi as ca
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from interfaces.msg import MotionCaptureState

from .load_cable_dynamics import (LoadCableDynamics, observed_state_indices,
                                  LOAD_DIM, CABLE_DIM)
from .planner_ocp import generate_load_ocp, nominal_hover_state

# World geometry, must match the world SDF. All overridable as ROS params.
N_DRONES      = 3
LOAD_MASS     = 0.4
LOAD_INERTIA  = [1.67e-3, 1.67e-3, 3.33e-3]
CABLE_LEN     = 0.6
ATTACH_RADIUS = 0.08
ATTACH_Z      = 0.025          # attach height above load CoG, load frame
DRONE_MASS    = 0.6
PLANNER_HZ    = 10.0

# Reference horizon. Defined here, not read off the OCP, so kinematic mode gets
# them without building the solver and the two modes cannot drift apart.
PLAN_N        = 20
PLAN_TF       = 2.0           # s, so node dt 0.1. Tracker expects N+1 nodes.
STEADY_ITERS  = 5             # SQP iters/cycle; 1 RTI step can't track the taut
                              # transition online

# Creep takeoff (phase 1), used while the cables are still slack.
CREEP_VEL     = 0.10          # m/s rise
CREEP_LEAD    = 0.10          # m z lead held while grounded, to initiate climb
LIFTOFF_MARGIN = 0.05         # m above spawn before the ramp starts
TAUT_SWITCH_GATE = 0.95       # hand over once every cable is at least this taut

TARGET_Z      = 0.6           # load hover height
LIFT_RAMP_VEL = 0.05          # m/s load lift rate after handover

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
LAND_VEL      = 0.20          # m/s descent rate, faster than the gentle lift

# Rigid ground-start handover. The measured elevation is allowed to lag the swept
# reference: the arc only asymptotes onto the target and the drones track just
# behind it, so demanding the exact target means handover never fires. Latching a
# few degrees low is cheap since tension comes from the measured geometry
# (40 deg needs 2.03 N/drone vs 1.85 N at 45).
HANDOVER_ELEV_TOL = 5.0       # deg the measurement may lag
HANDOVER_SETTLE_S = 1.0       # s within tolerance before latching
HANDOVER_TIMEOUT_S = 6.0      # s after the sweep ends, latch regardless

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
CABLE_ENGAGE_HEIGHT = 0.15    # m the load must lift before FULL tension FF
                              # engages. A taut cable to a grounded load bears
                              # ~0 tension; feeding a_cable then makes the tracker
                              # tilt out to fight a phantom pull.


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

        # Geometry and mode params, overriding the module defaults per world.
        # Must match the world SDF and the fleet manager's num_drones: the
        # planner sizes the attach ring and divides load tension by this, so a
        # mismatch mis-scales every drone's feedforward.
        self.n = int(self.declare_parameter('num_drones', N_DRONES).value)
        self._warn_gate_mismatch = False   # set after both params are read
        self.cable_len = float(
            self.declare_parameter('cable_len', CABLE_LEN).value)
        self.attach_radius = float(
            self.declare_parameter('attach_radius', ATTACH_RADIUS).value)
        self.attach_z = float(
            self.declare_parameter('attach_z', ATTACH_Z).value)
        # Must match the payload mass in the world SDF: cable tension is sized
        # off this, so a mismatch scales every drone's tension FF.
        self.load_mass = float(
            self.declare_parameter('load_mass', LOAD_MASS).value)
        # Hand over on cable ELEVATION (deg) instead of cable length. A rigid rod
        # is always exactly cable_len, so the length gate reads 1.0 from spawn and
        # hands over at ~0 deg, where tension mg/(n sin_elev) is effectively
        # infinite. Set for ground-start rigid worlds; 45 matches the elevated
        # ones. 0 = off, use the length gate (correct for soft cables).
        self.handover_elev_deg = float(
            self.declare_parameter('handover_elev_deg', 0.0).value)
        # Seconds to hold the latched config after handover before lifting.
        # Without it two transients land in the same cycle: the position
        # reference steps to the drones' actual pose (so the error the MPC was
        # fighting vanishes and it must unwind), and the lift starts pulling the
        # payload off the ground. Ground starts want ~2 s; 0 = off.
        self.handover_settle_s = float(
            self.declare_parameter('handover_settle_s', 0.0).value)
        self._settle_left = 0.0
        # Cables already taut at spawn (elevated world): skip the creep phase.
        self.start_taut = bool(
            self.declare_parameter('start_taut', False).value)
        # Load target rises from the handover height at lift_ramp_vel to
        # target_z. Params, so lift_ramp_vel:=0.0 gives a hold test.
        self.target_z = float(
            self.declare_parameter('target_z', TARGET_Z).value)
        self.lift_ramp_vel = float(
            self.declare_parameter('lift_ramp_vel', LIFT_RAMP_VEL).value)
        # See the module docstring. Default matches the launch file so a
        # standalone run doesn't silently get the unstable coupled path (and its
        # ~50 s solver build).
        self.planner_mode = str(
            self.declare_parameter('planner_mode', 'kinematic').value)
        # ff_gate_mode: how the cable/thrust feedforward is gated in.
        #   'airborne' ramp FF in as the load lifts off. Needed for soft cables,
        #              where feeding full tension while slack destabilises the
        #              tracker.
        #   'taut'     full FF as soon as the cable is geometrically taut (=1 from
        #              spawn for rigid cables), so the drones lift from the start
        #              rather than sliding inward or running ahead of the ramp.
        self.ff_gate_mode = str(
            self.declare_parameter('ff_gate_mode', 'airborne').value)
        # Load reference trajectory (kinematic mode). Once the lift tops out at
        # target_z the whole taut formation translates laterally, which preserves
        # the cable geometry so the cached FF stays valid.
        #   'hover'  no lateral motion, lift then hold.
        #   'line_x' continuous shuttle 0 -> traj_distance -> 0 until LAND.
        #            Sinusoidal, so traj_speed is the peak (mid-stroke) speed.
        #   'circle' horizontal circle of traj_radius at traj_speed.
        # Keep traj_speed slow: lateral accel is not fed forward, so fast motion
        # would need cable tilt this open-loop translation cannot model.
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
        # touchdown detector state (see _touchdown_stalled)
        self._land_prev_max_z = None
        self._land_stall_ct = 0
        self._land_cycles = 0
        # Separate from lift_ramp_vel so a slow takeoff doesn't force a slow land.
        self.land_vel = float(self.declare_parameter('land_vel', LAND_VEL).value)
        # Ground start + 'taut' contradict each other: the load is still on the
        # floor at handover so the cable bears ~no tension, but 'taut' engages
        # full FF immediately. Since the whole FF is gate-blended, the attitude
        # reference steps level -> ~13 deg in one cycle, i.e. a step in commanded
        # body rate, seen as a lurch. 'airborne' ramps it in with the lift.
        if self.handover_elev_deg > 0.0 and self.ff_gate_mode == 'taut':
            self.get_logger().warn(
                "[planner] handover_elev_deg is set (ground start) but "
                "ff_gate_mode='taut': the feedforward will engage as a STEP at "
                "handover while the payload is still grounded, which lurches the "
                "drones. Use ff_gate_mode:=airborne for ground-start worlds.")
        self.get_logger().info(
            f'[planner] geometry: n={self.n} cable_len={self.cable_len:.3f} '
            f'attach_radius={self.attach_radius:.3f} attach_z={self.attach_z:.3f} '
            f'load_mass={self.load_mass:.3f} '
            f'start_taut={self.start_taut} target_z={self.target_z:.3f} '
            f'lift_ramp_vel={self.lift_ramp_vel:.3f} mode={self.planner_mode} '
            f'ff_gate_mode={self.ff_gate_mode} load_traj={self.load_traj} '
            f'traj_speed={self.traj_speed:.3f} traj_distance={self.traj_distance:.3f} '
            f'traj_radius={self.traj_radius:.3f}')

        self.rho = [np.array([self.attach_radius * np.cos(2 * np.pi * k / self.n),
                              self.attach_radius * np.sin(2 * np.pi * k / self.n),
                              self.attach_z]) for k in range(self.n)]

        self.dyn = LoadCableDynamics(
            self.n, self.load_mass, LOAD_INERTIA, [self.cable_len] * self.n,
            self.rho, DRONE_MASS)
        # states pinned at OCP node 0 (observed): load pose/twist + cable dirs s_i.
        # Must match idxbx_0 in generate_load_ocp. r_i/tensions stay free.
        self._obs_idx = observed_state_indices(self.n)
        # Build the OCP only if we will solve it. acados recompiles it every
        # launch with no freshness check (~50 s for n=4), and kinematic mode never
        # solves it. Building it anyway published no references for the whole
        # compile, so the drones acknowledged TAKEOFF and then sat armed-idle
        # until it finished (the tracker holds while planner_ref_pos is None).
        if self.planner_mode == 'coupled':
            self.get_logger().info(
                f'[planner] building OCP (nx={self.dyn.nx}, nu={self.dyn.nu}) — '
                f'acados recompiles every launch, this takes a while...')
            self.ocp, self.solver = generate_load_ocp(
                self.dyn, N=PLAN_N, tf=PLAN_TF)
            self.N = self.ocp.solver_options.N_horizon
            self.dt = self.ocp.solver_options.tf / self.N
        else:
            self.ocp, self.solver = None, None
            self.N, self.dt = PLAN_N, PLAN_TF / PLAN_N
            self.get_logger().info(
                f'[planner] kinematic mode — skipping the coupled OCP build '
                f'(N={self.N}, dt={self.dt:.3f})')

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
        self.drone_vel = [None] * self.n       # world velocity from mocap twist
        self.last_X = None                     # previous solution (nx, N+1)
        self.hover_xy = None                   # captured load x,y for the reference
        self.first_solve = True
        self._recover = False                  # reseed+reconverge after a failed solve
        self.phase = 'creep'                   # 'creep' (slow rise to taut) -> 'planner'
        self.creep_anchor = None               # latched spawn xy/z per drone
        self.arc_anchor = None                 # rigid arc creep: (attach, radial)
        self.arc_theta0 = None                 # per-drone spawn elevation (rad)
        self.arc_theta = 0.0                   # swept elevation reference (rad)
        self._arc_wait = 0.0                   # s since the arc sweep finished
        self._arc_hold = 0.0                   # s measured elev held in tolerance
        self.creep_climb = 0.0                 # accumulated creep climb height (m)
        self.lifted_off = False                # gate the creep climb on real liftoff
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
        self.get_logger().info('[planner] ready, waiting for mocap...')

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

    # x_init assembly
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
        # Overwrite each cable DIRECTION s_i from measured geometry (drone->load, no
        # differentiation, clean). The cable RATES r_i and everything above them
        # (rd_i, rdd_i, t_i, td_i) stay RESAMPLED from the previous solution -- per
        # the paper (Fig 8), which resamples these rather than differentiating
        # estimator values, since numerical differentiation is too noisy. (An
        # earlier version measured r_i from drone-velocity differences; that fed
        # exactly that jitter into the pinned node 0 and the OCP fought it.)
        for i in range(self.n):
            b = LOAD_DIM + CABLE_DIM * i
            attach = p + R @ self.rho[i]
            d = attach - self.drone_pos[i]              # points drone -> load
            nrm = np.linalg.norm(d)
            if nrm > 1e-6:
                x[b:b + 3] = d / nrm
        return x

    def _yref_at(self, k):
        """Stage-k tracking reference (paper Eq 6, x_{k,ref}).

        The height is ramped ALONG the horizon and the lift velocity is fed into the
        velocity reference, so the OCP plans a coordinated climb. Feeding zero
        velocity against a rising setpoint made the load lag the target, and the
        planner ratcheted cable tension up trying to catch it (to the point the QP
        went infeasible). An agile time-varying load reference drops into the same
        position/velocity slots. The height ramp is decoupled from the measured
        load height (a fixed schedule from lift_z0), so a dip cannot lower the
        target and become positive feedback.
        """
        x0, y0 = self.hover_xy
        t_nom = self.dyn.m * 9.81 / (self.n * np.sin(np.deg2rad(45.0)))
        z_base = min(self.target_z, self.lift_z0 + self.lift_progress)
        vz = float(self._lift_vel)                 # signed lift rate this cycle
        z_k = z_base + vz * self.dt * k            # ramp the height along the horizon
        if vz >= 0.0:
            z_k = min(z_k, self.target_z)
            vz_k = 0.0 if z_k >= self.target_z - 1e-6 else vz
        else:
            vz_k = vz                              # descending (LAND): keep the rate
        pose = [x0, y0, z_k, 0.0, 0.0, vz_k, 0, 0, 0, 0, 0, 0]  # p, v, e_att, w
        return np.array(pose + [t_nom] * self.n
                        + [0.0] * (3 * self.n)     # r_vec ref (no cable swing)
                        + [0.0] * self.dyn.nu)

    # Plan step
    def _plan(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return

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
            if self.handover_elev_deg > 0.0:
                self._publish_arc_creep_refs()
            else:
                self._publish_creep_refs(gates)
            if self.handover_elev_deg > 0.0:
                # Rigid ground start: wait for the rod to rotate up to a liftable
                # angle (the length gate would hand over at ~0 deg). Only test
                # once the swept reference is done; before that the drones are
                # still on their way up.
                ref_done = self.arc_theta >= np.deg2rad(self.handover_elev_deg) - 1e-9
                sins = [self._cable_sin_elev(i) for i in range(self.n)]
                lo = np.degrees(np.arcsin(np.clip(min(sins), -1.0, 1.0)))
                if ref_done:
                    self._arc_wait += 1.0 / PLANNER_HZ
                    want = np.deg2rad(self.handover_elev_deg - HANDOVER_ELEV_TOL)
                    if min(sins) >= np.sin(want):
                        self._arc_hold += 1.0 / PLANNER_HZ
                    else:
                        self._arc_hold = 0.0
                    if self._arc_hold >= HANDOVER_SETTLE_S:
                        self._enter_planner_phase(
                            f'elevation {lo:.1f}deg reached '
                            f'(target {self.handover_elev_deg:.0f}, '
                            f'tol {HANDOVER_ELEV_TOL:.0f})')
                    elif self._arc_wait >= HANDOVER_TIMEOUT_S:
                        # Never strand the fleet mid-creep. Tension comes from
                        # the measured geometry, so a shallower angle still
                        # works, just heavier.
                        self.get_logger().warn(
                            f'[planner] arc creep timed out {self._arc_wait:.1f}s '
                            f'after the sweep ended; measured elevation only '
                            f'{lo:.1f}deg vs target {self.handover_elev_deg:.0f}. '
                            f'Latching anyway.')
                        self._enter_planner_phase(f'elevation timeout at {lo:.1f}deg')
            elif all(g >= TAUT_SWITCH_GATE for (g, _d) in gates):
                self._enter_planner_phase('cables taut')
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
        q_ref = np.array([1.0, 0.0, 0.0, 0.0])

        for k in range(self.N):
            self.solver.set(k, 'yref', self._yref_at(k))
            self.solver.set(k, 'p', q_ref)
        self.solver.set(self.N, 'yref', self._yref_at(self.N)[:-self.dyn.nu])
        self.solver.set(self.N, 'p', q_ref)

        # Reseed all nodes from x_init on the first solve, OR when recovering from a
        # failed solve (the previous warm start is NaN/garbage and would poison this
        # one). Otherwise keep the warm start from last_X.
        if self.first_solve or self._recover:
            for k in range(self.N + 1):
                self.solver.set(k, 'x', x_init)
        # Pin only the observed states at node 0 (see _obs_idx / idxbx_0). The full
        # x_init still seeds the warm start above; the unobserved tensions and cable
        # rates are left free so they cannot ratchet against a measured pose.
        self.solver.set(0, 'lbx', x_init[self._obs_idx])
        self.solver.set(0, 'ubx', x_init[self._obs_idx])

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
            # Shuttles continuously - like 'hover' it never self-completes, so
            # there is no auto-descent. End the run with a LAND command.
            return False
        if self.load_traj == 'circle':
            w = self.traj_speed / max(self.traj_radius, 1e-6)
            return self.traj_t >= 2.0 * np.pi / w
        return False

    def _load_offset(self):
        """Lateral (x, y) offset + velocity of the LOAD reference at the current
        traj_t. The whole taut formation is translated by this, so the load
        follows it. Returns (dx, dy, vx, vy); zero until the lift has topped out."""
        t = self.traj_t
        # At t=0 the velocity terms are not zero (vx = traj_speed*cos(0)), so the
        # drones were handed a full-speed reference at startup while the position
        # reference sat still. Hold at zero until the trajectory starts.
        if t <= 0.0:
            return 0.0, 0.0, 0.0, 0.0
        if self.load_traj == 'line_x':
            # Shuttle along +x, 0 -> traj_distance -> 0, repeating until LAND.
            # Sinusoidal rather than a triangle wave: constant speed reverses
            # velocity instantly at each end, and with lateral accel not fed
            # forward the payload takes that step as a pendulum kick. The
            # half-cosine has continuous velocity and accel and starts from rest.
            # w gives peak speed traj_speed; period = pi*traj_distance/traj_speed.
            d = max(self.traj_distance, 1e-6)
            w = 2.0 * self.traj_speed / d
            dx = 0.5 * d * (1.0 - np.cos(w * t))
            vx = 0.5 * d * w * np.sin(w * t)
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
        dx, dy, vx, vy = self._load_offset()
        x0 = float(self.hover_xy[0] + dx)
        y0 = float(self.hover_xy[1] + dy)
        msg = Float64MultiArray()
        msg.data = [x0, y0, float(z_des)]
        self.load_ref_pub.publish(msg)

        # Horizon path for RViz. Extrapolated exactly the way the drone
        # references are (_publish_kinematic_refs): constant lateral velocity
        # from the trajectory and the current lift rate, held over N+1 nodes.
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        z_cap = self.target_z if self.lift_z0 is not None else z_des
        for k in range(self.N + 1):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x0 + vx * self.dt * k
            ps.pose.position.y = y0 + vy * self.dt * k
            ps.pose.position.z = min(z_cap, z_des + self._lift_vel * self.dt * k)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.load_plan_pub.publish(path)

    def _enter_planner_phase(self, reason):
        """Transition creep -> coupled planner: latch the lift-ramp start height
        and force a hard reconverge from x_init on the first solve."""
        self.phase = 'planner'
        self._recover = True              # hard-reconverge from x_init on entry
        self.lift_z0 = float(self.load_state[2])   # ramp lift from here
        self.lift_progress = 0.0
        self._lift_t = 0.0                         # restart the lift easing
        self._settle_left = self.handover_settle_s
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
            # Blend the whole FF by the load-bearing gate, not just the cable
            # term: gate 0 (load grounded) gives a level unloaded hover FF, gate 1
            # the loaded tilted one. Holding a_thrust at the loaded value while
            # unloaded over-thrusts and over-tilts, driving the drone up and out
            # (the "sliding away" failure).
            a_i = (1.0 - gate) * a_unloaded + gate * self.kin_athrust[i]
            ac_i = gate * self.kin_acable[i]
            nodes = []
            for k in range(self.N + 1):
                x_off = dx + vx * self.dt * k
                y_off = dy + vy * self.dt * k
                z_off = self.lift_progress + lift_vel * self.dt * k
                p_i = (base[0] + x_off, base[1] + y_off, base[2] + z_off)
                nodes.append((p_i, (vx, vy, lift_vel), a_i, ac_i))
            self._publish_ref(i, nodes)

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

    def _publish_arc_creep_refs(self):
        """Phase-1 takeoff for a RIGID rod starting near-horizontal.

        A rigid rod fixes |drone - attach|. So commanding the drones straight up
        while holding their spawn xy (the soft-cable creep) cannot rotate the rod
        at all -- the horizontal leg stays put, the vertical leg is then pinned by
        the rod length, and the elevation never leaves its spawn value. The drones
        just try to drag the payload up at an angle needing ~13 N per drone.

        Instead sweep each drone along the ARC the rod actually permits, pivoting
        about its (grounded) attach point: elevation ramps from the spawn angle up
        to handover_elev_deg while the radius shrinks to match. The payload stays
        on the ground throughout, so there is no tension to fight, and at handover
        the geometry is the same liftable ~45 deg the elevated worlds spawn with.
        """
        if self.arc_anchor is None:
            ls = self.load_state
            R = quat_to_rot_np(ls[3:7])
            self.arc_anchor, self.arc_theta0 = [], []
            for i in range(self.n):
                attach = ls[0:3] + R @ self.rho[i]
                d = self.drone_pos[i] - attach
                horiz = np.array([d[0], d[1], 0.0])
                hn = float(np.linalg.norm(horiz))
                radial = horiz / hn if hn > 1e-6 else np.array([1.0, 0.0, 0.0])
                self.arc_anchor.append((attach.copy(), radial))
                self.arc_theta0.append(float(np.arctan2(d[2], hn)))
            self.arc_theta = float(np.mean(self.arc_theta0))
            self.get_logger().info(
                f'[planner] rigid arc creep: sweeping {np.degrees(self.arc_theta):.1f}'
                f' -> {self.handover_elev_deg:.1f} deg about the grounded attach '
                f'points (rod {self.cable_len:.2f} m)')

        target = np.deg2rad(self.handover_elev_deg)
        dtheta = CREEP_VEL / max(self.cable_len, 1e-6) / PLANNER_HZ
        if not self.lifted_off:
            for i in range(self.n):
                z_spawn = (self.arc_anchor[i][0][2]
                           + self.cable_len * np.sin(self.arc_theta0[i]))
                if self.drone_pos[i][2] > z_spawn + LIFTOFF_MARGIN:
                    self.lifted_off = True
                    break
        else:
            self.arc_theta = min(target, self.arc_theta + dtheta)

        th = self.arc_theta
        thd = dtheta * PLANNER_HZ if th < target else 0.0
        for i in range(self.n):
            attach, radial = self.arc_anchor[i]
            L = self.cable_len
            nodes = []
            for k in range(self.N + 1):
                a = min(target, th + thd * self.dt * k)
                p_i = attach + L * (np.cos(a) * radial
                                    + np.array([0.0, 0.0, np.sin(a)]))
                w = thd if a < target else 0.0
                v_i = L * w * (-np.sin(a) * radial
                               + np.array([0.0, 0.0, np.cos(a)]))
                if not self.lifted_off:
                    p_i = p_i + np.array([0.0, 0.0, CREEP_LEAD])
                    v_i = np.zeros(3)
                # level hover thrust, no cable term: the load is still grounded
                nodes.append((p_i, v_i, (0.0, 0.0, self.dyn.g), (0.0, 0.0, 0.0)))
            self._publish_ref(i, nodes)

        self._diag_ctr = getattr(self, '_diag_ctr', 0) + 1
        if self._diag_ctr % int(max(PLANNER_HZ, 1)) == 0:
            meas = '  '.join(
                f"d{i}:{np.degrees(np.arcsin(np.clip(self._cable_sin_elev(i),-1,1))):.1f}"
                for i in range(self.n))
            self.get_logger().info(
                f"[planner arc-creep] ref={np.degrees(th):.1f}deg "
                f"target={self.handover_elev_deg:.0f}deg "
                f"(latch >= {self.handover_elev_deg - HANDOVER_ELEV_TOL:.0f}deg "
                f"for {HANDOVER_SETTLE_S:.0f}s; held {self._arc_hold:.1f}s)  "
                f"measured: {meas}")

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
        # Don't accumulate climb until the drones leave the ground: the planner
        # runs while they are still disarmed, so the reference would run metres
        # above them and they would rocket up when armed. Hold a small constant
        # lead while grounded so takeoff still initiates.
        if not self.lifted_off:
            self.creep_climb = CREEP_LEAD
            if any(self.drone_pos[i][2] > self.creep_anchor[i][2] + LIFTOFF_MARGIN
                   for i in range(self.n)):
                self.lifted_off = True
        else:
            self.creep_climb += CREEP_VEL / PLANNER_HZ

        for i in range(self.n):
            ax, ay, az = self.creep_anchor[i]
            nodes = []
            for k in range(self.N + 1):
                z = az + self.creep_climb + CREEP_VEL * self.dt * k
                # level hover thrust, no cable term: cables still slack
                nodes.append(((ax, ay, z), (0.0, 0.0, CREEP_VEL),
                              (0.0, 0.0, self.dyn.g), (0.0, 0.0, 0.0)))
            self._publish_ref(i, nodes)

        self._diag_ctr = getattr(self, '_diag_ctr', 0) + 1
        if self._diag_ctr % int(max(PLANNER_HZ, 1)) == 0:
            if self.handover_elev_deg > 0.0:
                s = '  '.join(
                    f"d{i}:elev={np.degrees(np.arcsin(np.clip(self._cable_sin_elev(i),-1,1))):.1f}deg"
                    for i in range(self.n))
                s += f"  (handover at {self.handover_elev_deg:.0f}deg)"
            else:
                s = '  '.join(f"d{i}:dist={d:.2f} gate={g:.2f}"
                              for i, (g, d) in enumerate(gates))
            self.get_logger().info(f"[planner creep] vz={CREEP_VEL:.2f}  {s}")

    def _cable_sin_elev(self, i):
        """sin of the cable's elevation above horizontal, from the measured
        geometry. This is the quantity that sets tension (t = mg/(n sin_elev)),
        so it -- not cable length -- is what decides whether the configuration
        can actually lift the load."""
        ls = self.load_state
        R = quat_to_rot_np(ls[3:7])
        attach = ls[0:3] + R @ self.rho[i]
        d = attach - self.drone_pos[i]
        nrm = float(np.linalg.norm(d))
        if nrm < 1e-6:
            return 0.0
        return float(-d[2] / nrm)          # drone above attach -> positive

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
            nodes = []
            for k in range(self.N + 1):
                xk = X[:, k]
                nodes.append((
                    np.array(self.pos_fun[i](xk)).flatten(),
                    np.array(self.vel_fun[i](xk)).flatten(),
                    np.array(self.acc_fun[i](xk)).flatten(),
                    gate * np.array(self.cable_fun[i](xk)).flatten()))
            self._publish_ref(i, nodes)

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
                d = (ls[0:3] + R @ self.rho[i]) - self.drone_pos[i]
                nd = np.linalg.norm(d)
                elevs.append(np.degrees(np.arcsin(np.clip(-d[2] / max(nd, 1e-6), -1, 1))))
            e = ' '.join(f"{x:.0f}" for x in elevs)
            self.get_logger().info(
                f"[planner cable] L={self.cable_len:.2f} load_z={self.load_state[2]:.2f} "
                f"z_tgt={z_tgt:.2f} tilt={tilt:.1f}deg elev=[{e}]  {s}")


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
