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

from .geometry import balanced_tensions, rot_align, rot_z, yaw_quat


def blend_refs(s_from, s_nom, t_from, t_nom, a):
    """Per-slot cable-direction and tension references a fraction `a` of the way from
    a starting state to nominal: directions are normalised after the linear blend,
    tensions blend linearly. a <= 0 gives the start, a >= 1 the nominal."""
    a = float(min(1.0, max(0.0, a)))
    s_out, t_out = [], []
    for sf, sn, tf, tn in zip(s_from, s_nom, t_from, t_nom):
        v = (1.0 - a) * np.asarray(sf, float) + a * np.asarray(sn, float)
        nv = float(np.linalg.norm(v))
        s_out.append(v / nv if nv > 1e-9 else np.asarray(sn, float))
        t_out.append((1.0 - a) * float(tf) + a * float(tn))
    return s_out, t_out


def unload_tensions(rho, s_dirs, load_mass, k, t_leave, rod_mass=0.0, rod_lam=0.0, g=9.81, t_min=0.1):
    """Tension references that hand slot k's share to the others before it lets go (the
    soft detach): slot k at t_leave, the rest at the split that holds the load level WITH
    slot k's remaining pull counted (the static balance of balanced_tensions, least squares
    from the equal split). With the directions unchanged a linear blend to it is balanced
    all the way, because both ends are. After the release the n-1 builder's own split
    differs from this by slot k's t_leave share; the post-release blend covers that."""
    n = len(rho)
    keep = [i for i in range(n) if i != k]
    s = [np.asarray(v, float).reshape(3) for v in s_dirs]
    r = [np.asarray(v, float).reshape(3) for v in rho]
    b = np.array([0.0, 0.0, -float(load_mass) * g, 0.0, 0.0, 0.0])
    if rod_mass > 0.0:
        G = np.array([0.0, 0.0, -g])
        for i in range(n):
            R_i = rod_mass * (rod_lam * float(G @ s[i]) * s[i] + (1.0 - rod_lam) * G)
            b[0:3] += R_i
            b[3:6] += np.cross(r[i], R_i)
    f_k = float(t_leave) * s[k]
    b = b - np.concatenate([f_k, np.cross(r[k], f_k)])
    A = np.stack([np.concatenate([s[i], np.cross(r[i], s[i])]) for i in keep], axis=1)
    sz = -np.mean([s[i][2] for i in keep])
    t_eq = np.full(len(keep), -b[2] / max(len(keep) * sz, 1e-9))
    t = t_eq + np.linalg.pinv(A) @ (b - A @ t_eq)
    out = [0.0] * n
    for i, v in zip(keep, t):
        out[i] = float(max(v, t_min))
    out[k] = float(t_leave)
    return out


def rebalance_incumbents(rho, s_dirs, t, load_mass, fixed=(-1,), g=9.81, t_min=0.1):
    """Keep the `fixed` slots' tensions and re-solve the others (least squares, closest to
    the given values) so the blended references still hold the load up and level:
        sum_i t_i s_iz = -m g,   (sum_i rho_i x (t_i s_i))_{x,y} = 0.
    The horizontal force and yaw rows are left out: three incumbents cannot also zero
    them, and weighting them in leaves a roll moment (0.13 N m midway on 1/3/5/9).
    A linear blend is balanced at both ends but not in between when a slot's direction
    rotates while its tension ramps (the attach newcomer: vertical -> apex)."""
    n = len(rho)
    fixed = {i % n for i in fixed}
    free = [i for i in range(n) if i not in fixed]
    s = [np.asarray(v, float).reshape(3) for v in s_dirs]
    r = [np.asarray(v, float).reshape(3) for v in rho]
    col = lambda i: np.concatenate([s[i][2:3], np.cross(r[i], s[i])[0:2]])
    b = np.array([-float(load_mass) * g, 0.0, 0.0])
    for i in fixed:
        b = b - col(i) * float(t[i])
    A = np.stack([col(i) for i in free], axis=1)
    t0 = np.array([float(t[i]) for i in free])
    tf = t0 + np.linalg.pinv(A) @ (b - A @ t0)
    out = [float(v) for v in t]
    for k, i in enumerate(free):
        out[i] = float(max(tf[k], t_min))
    return out


class ReferenceBuilder:
    def __init__(self, dyn, n, s_nom, dt, traj):
        self.dyn = dyn
        self.n = n
        self._s_nom = s_nom
        # per-drone nominal tensions that hold THIS layout level (equal on an even ring)
        self._t_nom = balanced_tensions(dyn.rho, s_nom, dyn.m, rod_mass=getattr(dyn, 'mr', 0.0),
                                        rod_lam=getattr(dyn, 'lam', 0.0))
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
        # Post-attach slew (2026-09-25): a welded newcomer's rod starts vertical and
        # unloaded; re-planning it straight to the 45 deg slot yanked the Gazebo
        # newcomer over (R0541). From start_blend() the per-slot direction and
        # tension references slew from the weld state to nominal over blend_s, the
        # incumbents' tension shares included (the OCP form of the network's
        # symmetric hand-out). Inactive (None) for every other flight.
        self._blend = None            # (s_from, t_from, T_s, elapsed_s)
        self.balanced_blend = False   # re-solve the incumbents' tensions along the slew
        # estimated constant external force on the load (N, world), 0 unless applied
        self.d_L = np.zeros(3)

    def start_blend(self, s_from, t_from, blend_s):
        """Slew the cable references from (s_from, t_from) -- per slot, s in the same
        frame as s_nom -- to nominal over blend_s seconds, advancing per update()."""
        if blend_s <= 0.0:
            self._blend = None
            return
        self._blend = ([np.asarray(v, float) for v in s_from],
                       [float(v) for v in t_from], float(blend_s), 0.0)

    def retarget(self, t_nom, blend_s):
        """Blend the tension references from where they are now (a running blend
        included) to new nominal tensions over blend_s, directions held: the soft
        detach's unload, and its cancel back to the full split."""
        s_now, t_now = self._refs_at(0)
        self._t_nom = [float(v) for v in t_nom]
        self.start_blend(s_now, t_now, blend_s)

    def blend_active(self):
        return self._blend is not None

    def _refs_at(self, k):
        """(s_nom-frame directions, tensions) for stage k, blended if a slew is on."""
        if self._blend is None:
            return self._s_nom, self._t_nom
        s_from, t_from, T, el = self._blend
        s_b, t_b = blend_refs(s_from, self._s_nom, t_from, self._t_nom, (el + self.dt * k) / T)
        if self.balanced_blend:
            t_b = rebalance_incumbents(self.dyn.rho, s_b, t_b, self.dyn.m)
        return s_b, t_b

    def set_yaw_datum(self, psi0):
        """Latch the payload's starting yaw as the reference datum (see psi0)."""
        self.psi0 = float(psi0)

    def update(self, hover_xy, lift_z0, lift_progress, target_z, lift_vel, traj_t,
               z_bias=0.0):
        """Refresh the lift schedule for this planner tick (call before solving).
        `z_bias`: the planner's bounded height integral (ZBias), added to the height
        reference AFTER the lift cap so it corrects a low hover as well as a high one."""
        self.z_bias = float(z_bias)
        if self._blend is not None:
            s_from, t_from, T, el = self._blend
            el += self.dt
            self._blend = None if el >= T else (s_from, t_from, T, el)
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
        z_base = min(self.target_z, self.lift_z0 + self.lift_progress) + getattr(self, 'z_bias', 0.0)
        vz = float(self._lift_vel)                 # signed lift rate this cycle
        z_k = z_base + vz * self.dt * k            # ramp the height along the horizon
        if vz >= 0.0:
            z_k = min(z_k, self.target_z + getattr(self, 'z_bias', 0.0))
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
        # an external force on the load is countered like an acceleration (it is one, F/m)
        g_eff = np.array([-ax, -ay, -self.dyn.g]) + self.d_L / self.dyn.m
        g_eff_mag = float(np.linalg.norm(g_eff))
        R_tilt = rot_align(np.array([0.0, 0.0, -1.0]), g_eff)
        # The attach points rotate with the load yaw, so the nominal cable directions
        # rotate with it too (about world z): yaw the formation, then tilt onto g_eff.
        # The yaw is the latched placement datum psi0 plus, for 'spin', the commanded
        # load rotation (0 for every other trajectory).
        R_form = R_tilt @ rot_z(self.psi0 + yaw)
        s_k, t_k = self._refs_at(k)
        s_ref = []
        for i in range(self.n):
            s_ref.extend((R_form @ s_k[i]).tolist())
        t_ref = [ti * g_eff_mag / self.dyn.g for ti in t_k]
        return np.array(pose + s_ref              # cable directions (flatness)
                        + t_ref                   # cable tensions (flatness), layout-balanced
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
        pose = [float(p[0]), float(p[1]), float(p[2]),
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        R_yaw = rot_z(self.psi0)
        s_0, t_0 = self._refs_at(0)
        s_ref = []
        for i in range(self.n):
            s_ref.extend((R_yaw @ s_0[i]).tolist())
        return np.array(pose + s_ref + list(t_0)
                        + [0.0] * (3 * self.n) + [0.0] * self.dyn.nu)
