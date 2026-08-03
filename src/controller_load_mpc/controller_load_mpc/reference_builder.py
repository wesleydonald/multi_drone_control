"""
reference_builder.py
--------------------
Turns the lift schedule + load trajectory into the per-node OCP tracking reference,
factored out of the planner node. This is the "what should the load be doing at
horizon node k" half of the planner (paper Eq 6); the solver consumes yref_at /
q_ref_at as callables and never needs to know about the flight phase.

The lift schedule (hover xy, ramp height, lift velocity, trajectory clock) is node
state that changes every tick, so the node calls update() with it before each
planner solve; yref_at(k)/q_ref_at(k) then read it, keeping the plain callable
signature the solver expects. hold_yref() is the static "keep the current pose"
reference used to prime the solver warm during creep.
"""
import numpy as np

from .geometry import rot_align, rot_z, yaw_quat


class ReferenceBuilder:
    def __init__(self, dyn, n, s_nom, dt, traj):
        self.dyn = dyn
        self.n = n
        self._s_nom = s_nom
        self.dt = dt
        self.traj = traj
        # Load YAW DATUM: the payload's measured yaw, latched once by the node
        # (set_yaw_datum) before the first solve. s_nom is built from the attach
        # ring, which lives in the LOAD frame, and the load attitude reference is
        # about world z -- so both must be expressed about the yaw the rig was
        # actually placed at. Left at 0.0 the references are world-aligned, which
        # commands the whole formation to rotate the payload back to yaw 0 on
        # takeoff (up to half a slot pitch of unwanted rotation, dragging every
        # drone around the ring with it).
        self.psi0 = 0.0
        # Per-tick lift schedule, set via update() before each planner solve.
        self.hover_xy = None
        self.lift_z0 = None
        self.lift_progress = 0.0
        self.target_z = 0.0
        self._lift_vel = 0.0
        self.traj_t = 0.0

    def set_yaw_datum(self, psi0):
        """Latch the payload's starting yaw as the reference datum (see psi0)."""
        self.psi0 = float(psi0)

    def update(self, hover_xy, lift_z0, lift_progress, target_z, lift_vel, traj_t):
        """Refresh the lift schedule for this planner tick (call before solving)."""
        self.hover_xy = hover_xy
        self.lift_z0 = lift_z0
        self.lift_progress = lift_progress
        self.target_z = target_z
        self._lift_vel = lift_vel
        self.traj_t = traj_t

    def yref_at(self, k):
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
        # Lateral load trajectory (line_x / circle), evaluated at THIS node's horizon
        # time so the whole 2 s window tracks the moving load, not just node 0. Gated
        # on traj_t > 0 (set once the lift tops out) so nothing drifts during the
        # climb; the trajectories start from rest so the first active cycle is smooth.
        dx = dy = vx = vy = ax = ay = 0.0
        yaw = yaw_rate = 0.0
        if self.traj_t > 0.0:
            t_k = self.traj_t + self.dt * k
            dx, dy, vx, vy = self.traj.offset_at(t_k)
            ax, ay = self.traj.accel_at(t_k)
            yaw, yaw_rate = self.traj.yaw_at(t_k)           # 'spin' only, else 0
        # p, v, e_att, w. The load yaw REFERENCE goes on the q_ref parameter (set per
        # node in _plan), so e_att stays 0 here; the yaw RATE is the load angular
        # velocity ref (world z), which drives the OCP to actually rotate the load.
        pose = [x0 + dx, y0 + dy, z_k, vx, vy, vz_k, 0, 0, 0, 0, 0, yaw_rate]
        # Flatness cable references. A load accelerating at a_load must have its cables
        # counter an EFFECTIVE gravity g_eff = g - a_load: the formation tilts toward
        # g_eff and the tension scales with |g_eff|. Feeding this anticipates the
        # maneuver (drives the cable-direction chain directly) instead of discovering
        # the tilt reactively from load-position error. At hover a_load=0 -> nominal
        # 45 deg directions + nominal tension, so this also pins the formation radius.
        g_eff = np.array([-ax, -ay, -self.dyn.g])
        g_eff_mag = float(np.linalg.norm(g_eff))
        R_tilt = rot_align(np.array([0.0, 0.0, -1.0]), g_eff)
        # The attach points rotate with the load yaw, so the nominal cable directions
        # rotate with it too (about world z): yaw the formation, then tilt onto g_eff.
        # The yaw is the latched placement datum psi0 plus, for 'spin', the commanded
        # load rotation (0 for every other trajectory).
        R_form = R_tilt @ rot_z(self.psi0 + yaw)
        s_ref = []
        for i in range(self.n):
            s_ref.extend((R_form @ self._s_nom[i]).tolist())
        t_ref = t_nom * g_eff_mag / self.dyn.g
        return np.array(pose + s_ref              # cable directions (flatness)
                        + [t_ref] * self.n        # cable tensions (flatness)
                        + [0.0] * (3 * self.n)    # r_vec ref (no cable swing)
                        + [0.0] * self.dyn.nu)

    def q_ref_at(self, k):
        """Stage-k load attitude reference (the q_ref acados parameter): hold the yaw
        the rig was placed at (psi0). For 'spin' the commanded load rotation is added
        on top, tracking the load yaw along the horizon so the OCP holds the rotating
        attitude while the yaw-rate term in yref_at drives the rotation."""
        yaw = 0.0
        if self.traj_t > 0.0:
            yaw, _ = self.traj.yaw_at(self.traj_t + self.dt * k)
        return yaw_quat(self.psi0 + yaw)

    def hold_yref(self, load_state):
        """Static hold reference for priming: keep the load at its current measured
        position with zero twist, cables at the nominal 45 deg taut directions (about
        the placement yaw datum, as in yref_at) and nominal tension. Same field layout
        as yref_at."""
        p = load_state[0:3]
        t_nom = self.dyn.m * 9.81 / (self.n * np.sin(np.deg2rad(45.0)))
        pose = [float(p[0]), float(p[1]), float(p[2]),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        R_yaw = rot_z(self.psi0)
        s_ref = []
        for i in range(self.n):
            s_ref.extend((R_yaw @ self._s_nom[i]).tolist())
        return np.array(pose + s_ref + [t_nom] * self.n
                        + [0.0] * (3 * self.n) + [0.0] * self.dyn.nu)
