"""
planner_node.py
---------------
Centralized cable-suspended load planner node (Sun et al. 2025).

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
from std_msgs.msg import Float64MultiArray
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

        self.n = N_DRONES
        self.rho = [np.array([ATTACH_RADIUS * np.cos(2 * np.pi * k / self.n),
                              ATTACH_RADIUS * np.sin(2 * np.pi * k / self.n),
                              ATTACH_Z]) for k in range(self.n)]

        self.dyn = LoadCableDynamics(
            self.n, LOAD_MASS, LOAD_INERTIA, [CABLE_LEN] * self.n,
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
        self.lifted_off = False                # gate the climb ramp on real liftoff
        self.lift_z0 = None                    # load height latched at handover
        self.lift_progress = 0.0               # ramped lift above lift_z0 (m)

        # subs
        self.create_subscription(
            MotionCaptureState, '/payload/motion_capture_state',
            self._payload_cb, 5)
        for i in range(self.n):
            self.create_subscription(
                MotionCaptureState, f'/drone_{i}/motion_capture_state',
                lambda msg, k=i: self._drone_cb(msg, k), 5)

        # pubs
        self.ref_pub = [self.create_publisher(
            Float64MultiArray, f'/drone_{i}/reference_trajectory', 5)
            for i in range(self.n)]

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
        z_target = min(TARGET_Z, self.lift_z0 + self.lift_progress)
        pose = [x0, y0, z_target, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # p, v, e_att, w
        y = np.array(pose + [t_nom] * self.n
                     + [0.0] * (3 * self.n)        # r_vec ref (no cable swing)
                     + [0.0] * self.dyn.nu)
        return y, y[:-self.dyn.nu]

    # ── plan step ───────────────────────────────────────────────────────────
    def _plan(self):
        if self.load_state is None or any(d is None for d in self.drone_pos):
            return

        # ── Phase 1: creep takeoff ───────────────────────────────────────────
        # The coupled OCP assumes taut cables. While the cables are slack the
        # drones must climb ~0.4 m to take up the slack; running the lift planner
        # through that makes them overshoot and snap the soft cable taut, which
        # the open-loop tracker cannot ride. Instead we bypass the planner and
        # creep each drone straight up slowly until every cable is taut, then hand
        # over. This keeps the slack->taut transition gentle (low velocity at
        # contact) so tension never spikes past the drones' thrust authority.
        gates = [self._cable_taut_gate(i) for i in range(self.n)]
        if self.phase == 'creep':
            self._publish_creep_refs(gates)
            if all(g >= TAUT_SWITCH_GATE for (g, _d) in gates):
                self.phase = 'planner'
                self._recover = True          # hard-reconverge from x_init on entry
                self.lift_z0 = float(self.load_state[2])   # ramp lift from here
                self.lift_progress = 0.0
                self.get_logger().info(
                    '[planner] cables taut — handing over to coupled load planner '
                    f'(lift from z={self.lift_z0:.2f})')
            return

        # planner phase: advance the time-based lift ramp (decoupled from load_z)
        self.lift_progress = min(TARGET_Z - self.lift_z0,
                                 self.lift_progress + LIFT_RAMP_VEL / PLANNER_HZ)

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
        """(gate, dist): `gate` in [0, 1] is how taut cable i physically is right
        now, from the current measured drone->attach distance `dist` vs the cable
        length. 0 while the cable is clearly slack (so we don't feed phantom tension
        into the tracker), ramping to 1 as it straightens. Mirrors the slack->taut
        transition the rigid OCP model cannot see."""
        ls = self.load_state
        R = quat_to_rot_np(ls[3:7])
        attach = ls[0:3] + R @ self.rho[i]
        dist = float(np.linalg.norm(attach - self.drone_pos[i]))
        d_lo = CABLE_TAUT_LO_FRAC * CABLE_LEN
        d_hi = CABLE_TAUT_HI_FRAC * CABLE_LEN
        gate = float(np.clip((dist - d_lo) / max(d_hi - d_lo, 1e-6), 0.0, 1.0))
        return gate, dist

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
            self.get_logger().info(f"[planner cable] L={CABLE_LEN:.2f}  {s}")


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
