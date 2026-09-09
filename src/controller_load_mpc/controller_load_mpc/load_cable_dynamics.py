"""
load_cable_dynamics.py
----------------------
CasADi model of the cable-suspended multi-lifting system from
Sun et al. 2025 (arXiv 2501.18802), Eqs (1)-(3), with the quadrotor kinematics
(Eq 5) and thrust (Eq 9) needed for the OCP path constraints and reference
extraction.

State (Eq 1), n = number of drones:
    x = [ p(3), v(3), q(4, wxyz), w(3),                       # rigid-body load
          { s_i(3), r_i(3), rd_i(3), rdd_i(3), t_i(1), td_i(1) } for i ]
        |-------------- 13 --------------|  + 14 per drone   ->  13 + 14n

Control (Eq 3):  u = [ { gamma_i(3), lambda_i(1) } for i ]    ->  4n
    gamma_i  = 3rd derivative of cable angular velocity (snap of direction)
    lambda_i = 2nd derivative of cable tension

Dynamics (Eqs 2-3):
    p_dot = v
    v_dot = -sum_i t_i s_i / m + g
    q_dot = 1/2 * q (x) [0; w]
    J w_dot = -w x Jw + sum_i t_i ( R(q)^T s_i  x  rho_i )
    s_dot_i  = r_i x s_i
    r_dot_i  = rd_i ;  rd_dot_i = rdd_i ;  rdd_dot_i = gamma_i
    t_dot_i  = td_i ;  td_dot_i = lambda_i

Cable direction s_i points FROM the quadrotor TO the load (per the paper), so the
cable force on the LOAD is -t_i s_i and the kinematic constraint is
    p_i = p + R(q) rho_i - l_i s_i        (Eq 5)

Structured like controller_mpc_multi/dynamics.py (QuadDynamics): each state block
has its own `*_dynamics()` method, assembled into a CasADi Function by
`load_cable_dynamics()` (the analogue of `quad_dynamics()`).
"""
import numpy as np
import casadi as cs

GRAVITY = cs.DM([0.0, 0.0, -9.81])

# per-drone state/control block sizes
LOAD_DIM = 13          # p(3) v(3) q(4) w(3)
CABLE_DIM = 14         # s(3) r(3) rd(3) rdd(3) t(1) td(1)
CTRL_DIM = 4           # gamma(3) lambda(1)


def observed_state_indices(n):
    """State indices that mocap observes cleanly and that MAY be hard-pinned at OCP
    node 0: the full load block p,v,q,w (0..12) and each cable DIRECTION s_i. The
    cable rates r_i and everything above (rd_i, rdd_i, t_i, td_i) are NOT included:
    they are unobservable / noisy-to-differentiate, are only warm-started by
    resampling the previous solution, and must stay FREE decision variables. Pinning
    the stale resampled tension against a measured pose the tracker didn't quite hit
    makes node 0 inconsistent, and the planned tension ratchets up cycle over cycle.
    """
    idx = list(range(LOAD_DIM))                     # p, v, q, w
    for i in range(n):
        b = LOAD_DIM + CABLE_DIM * i
        idx += [b, b + 1, b + 2]                    # s_i only
    return np.array(idx, dtype=int)


def quat_mul(q1, q2):
    """Hamilton product q1 (x) q2, both [w, x, y, z]."""
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return cs.vertcat(
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


class LoadCableDynamics:
    """Symbolic CasADi model for n cable-suspended quadrotors carrying a load.

    ATTACHMENT POINTS rho_i AND CABLE LENGTHS l_i ARE RUNTIME PARAMETERS, not baked
    constants. Only the fleet size n sets the problem dimensions, so ONE compiled
    solver per n covers any attachment geometry.

    That matters for two things this project needs (THESIS_PLAN §12.2):
      * a welded newcomer attaches WHEREVER ITS MAGNET LANDS, not at a nominal ring
        point, and a baked rho cannot describe that without a ~50 s rebuild you
        cannot do mid-flight;
      * handing the fleet back to the OCP after a reconfiguration needs the solver to
        accept the geometry the fleet actually has now.

    Masses and inertia stay baked: they do not change during a flight, and leaving
    them constant keeps the parameter vector small.

    `self.rho` / `self.l` remain the NOMINAL numeric values. They are the default
    parameter values and are what build-time constructs (the hover seed, the cache
    signature) read; the equations use the symbolic ones.
    """

    def __init__(self, n_drones, load_mass, load_inertia, cable_lengths,
                 attach_points, drone_mass):
        self.n = int(n_drones)
        self.m = float(load_mass)
        J = np.asarray(load_inertia, dtype=float)
        self.J = J if J.ndim == 2 else np.diag(J)      # accept diag or full 3x3
        self.Jinv = np.linalg.inv(self.J)
        self.l = [float(v) for v in cable_lengths]
        self.rho = [np.asarray(p, dtype=float).reshape(3) for p in attach_points]
        self.mi = ([float(drone_mass)] * self.n if np.isscalar(drone_mass)
                   else [float(v) for v in drone_mass])
        assert len(self.l) == self.n and len(self.rho) == self.n

        self.g = 9.81
        self.nx = LOAD_DIM + CABLE_DIM * self.n
        self.nu = CTRL_DIM * self.n

        self.x = cs.MX.sym('x', self.nx)
        self.u = cs.MX.sym('u', self.nu)
        # Runtime geometry: rho (3 per drone) then l (1 per drone).
        self.p_geom = cs.MX.sym('p_geom', 4 * self.n)
        self.rho_p = [self.p_geom[3 * i:3 * i + 3] for i in range(self.n)]
        self.l_p = [self.p_geom[3 * self.n + i] for i in range(self.n)]
        self._unpack()
        # load translational/rotational accelerations are shared by the load
        # dynamics and the quadrotor thrust constraint, so compute them once
        self.v_dot, self.w_dot = self._load_accel()
        self.f_expl = self._build_f_expl()

    # ---- helper functions --------------------------------------------------
    def q_to_rot_mat(self, q):
        """Rotation matrix from unit quaternion q = [w, x, y, z]."""
        w, x, y, z = q[0], q[1], q[2], q[3]
        return cs.vertcat(
            cs.horzcat(1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)),
            cs.horzcat(2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            cs.horzcat(2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)),
        )

    # ---- state/control accessors -------------------------------------------
    def _unpack(self):
        x = self.x
        self.p = x[0:3] # position, velocity, quarternion, angular velocity of payload
        self.v = x[3:6]
        self.q = x[6:10]
        self.w = x[10:13]
        self.R = self.q_to_rot_mat(self.q)
        self.s, self.r, self.rd, self.rdd, self.t, self.td = [], [], [], [], [], []
        for i in range(self.n):
            b = LOAD_DIM + CABLE_DIM * i
            self.s.append(x[b:b + 3]) # cable direction pointing from quadcopter 'i' to load
            self.r.append(x[b + 3:b + 6]) # cable angular velocity
            self.rd.append(x[b + 6:b + 9]) # cable angular acceleration
            self.rdd.append(x[b + 9:b + 12]) # cable angular snap
            self.t.append(x[b + 12]) # cable tension
            self.td.append(x[b + 13]) # change in cable tension
        self.gamma, self.lmbda = [], []
        for i in range(self.n):
            c = CTRL_DIM * i
            self.gamma.append(self.u[c:c + 3])
            self.lmbda.append(self.u[c + 3])

    # ---- load accelerations (shared by dynamics + thrust constraint) -------
    def _load_accel(self):
        v_dot = -sum((self.t[i] * self.s[i] for i in range(self.n)),
                     cs.MX.zeros(3)) / self.m + GRAVITY
        torque = sum((self.t[i] * cs.cross(self.R.T @ self.s[i], self.rho_p[i])
                      for i in range(self.n)), cs.MX.zeros(3))
        w_dot = self.Jinv @ (-cs.cross(self.w, self.J @ self.w) + torque)
        return v_dot, w_dot

    # ---- per-state-block dynamics ------------------------------------------
    def p_dynamics(self):
        return self.v

    def v_dynamics(self):
        return self.v_dot

    def q_dynamics(self):
        return 0.5 * quat_mul(self.q, cs.vertcat(0, self.w))

    def w_dynamics(self):
        return self.w_dot

    def cable_dynamics(self, i):
        return cs.vertcat(
            cs.cross(self.r[i], self.s[i]),  # s_dot
            self.rd[i],                       # r_dot
            self.rdd[i],                      # rd_dot
            self.gamma[i],                    # rdd_dot
            self.td[i],                       # t_dot
            self.lmbda[i],                    # td_dot
        )

    def _build_f_expl(self):
        return cs.vertcat(
            self.p_dynamics(), self.v_dynamics(), self.q_dynamics(), self.w_dynamics(),
            *[self.cable_dynamics(i) for i in range(self.n)])

    def geom_values(self, rho=None, l=None):
        """Numeric p_geom vector -- rho (3 per drone) then l (1 per drone).

        Defaults to the nominal geometry, so a solver created with these values
        behaves exactly as the old baked-constant model did."""
        r = self.rho if rho is None else [np.asarray(v, float).reshape(3)
                                          for v in rho]
        ln = self.l if l is None else [float(v) for v in l]
        assert len(r) == self.n and len(ln) == self.n
        return np.concatenate([np.concatenate(r), np.asarray(ln, float)])

    def load_cable_dynamics(self):
        """Full state derivative x_dot as a CasADi Function (analogue of
        QuadDynamics.quad_dynamics). Takes the geometry parameter vector."""
        return cs.Function('x_dot', [self.x, self.u, self.p_geom], [self.f_expl],
                           ['x', 'u', 'p_geom'], ['x_dot'])

    # ---- quadrotor kinematics (Eq 5 and derivatives) -----------------------
    def quad_position(self, i):
        return self.p + self.R @ self.rho_p[i] - self.l_p[i] * self.s[i]

    def quad_velocity(self, i):
        # d/dt(R rho_i) = R (w x rho_i);  d/dt(s_i) = r_i x s_i
        return (self.v + self.R @ cs.cross(self.w, self.rho_p[i])
                - self.l_p[i] * cs.cross(self.r[i], self.s[i]))

    def quad_accel(self, i):
        # d/dt[R(w x rho)] = R[ w_dot x rho + w x (w x rho) ]
        rot_term = self.R @ (cs.cross(self.w_dot, self.rho_p[i])
                             + cs.cross(self.w, cs.cross(self.w, self.rho_p[i])))
        # d/dt[r x s] = rd x s + r x (r x s)
        cab_term = (cs.cross(self.rd[i], self.s[i])
                    + cs.cross(self.r[i], cs.cross(self.r[i], self.s[i])))
        return self.v_dot + rot_term - self.l_p[i] * cab_term

    def thrust_vec(self, i):
        """Collective thrust force VECTOR of drone i, world frame (Eq 9, drag
        neglected). Points along the drone's body-z axis; its magnitude is the
        collective thrust. f_i = m_i (a_i - g) - t_i s_i."""
        return self.mi[i] * (self.quad_accel(i) - GRAVITY) - self.t[i] * self.s[i]

    def thrust(self, i):
        """Collective thrust magnitude of drone i (Eq 9, drag neglected)."""
        return cs.norm_2(self.thrust_vec(i))

    def cable_accel(self, i):
        """Acceleration imparted on drone i by its cable tension, world frame.

        The cable force on the drone is +t_i s_i (s_i points drone->load, so the
        taut cable pulls the drone toward the load). The per-drone tracker model
        works in acceleration units (no explicit mass), so we hand it the cable
        ACCELERATION a_cable_i = t_i s_i / m_i to add to its v_dynamics."""
        return self.t[i] * self.s[i] / self.mi[i]


if __name__ == "__main__":
    # quick self-test for the three_soft geometry (3 drones at 120deg, R_a=0.08)
    n = 3
    rho = [[0.08 * np.cos(2 * np.pi * k / 3), 0.08 * np.sin(2 * np.pi * k / 3), 0.025]
           for k in range(n)]
    dyn = LoadCableDynamics(n, load_mass=0.4, load_inertia=[1.67e-3, 1.67e-3, 3.33e-3],
                            cable_lengths=[0.42] * n, attach_points=rho, drone_mass=0.6)
    print(f"n={n}  nx={dyn.nx} (expect {13 + 14 * n})  nu={dyn.nu} (expect {4 * n})")
    f = cs.Function('f', [dyn.x, dyn.u], [dyn.f_expl])
    x0 = np.zeros(dyn.nx)
    x0[6] = 1.0  # quat w = 1
    for i in range(n):
        x0[13 + 14 * i:13 + 14 * i + 3] = [0, 0, 1]   # s_i = +z (unit)
        x0[13 + 14 * i + 12] = 1.5                      # t_i = 1.5 N (taut)
    xdot = np.array(f(x0, np.zeros(dyn.nu))).flatten()
    print("f_expl ok, xdot[:6] (load p_dot,v_dot) =", np.round(xdot[:6], 3))
    Ti = cs.Function('T', [dyn.x, dyn.u], [dyn.thrust(0)])
    print("thrust(drone0) at rest =", float(Ti(x0, np.zeros(dyn.nu))), "N")
