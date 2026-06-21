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
"""
import numpy as np
import casadi as ca

GRAVITY = ca.DM([0.0, 0.0, -9.81])

# per-drone state/control block sizes
LOAD_DIM = 13          # p(3) v(3) q(4) w(3)
CABLE_DIM = 14         # s(3) r(3) rd(3) rdd(3) t(1) td(1)
CTRL_DIM = 4           # gamma(3) lambda(1)


def skew(v):
    return ca.vertcat(
        ca.horzcat(0,      -v[2],  v[1]),
        ca.horzcat(v[2],    0,    -v[0]),
        ca.horzcat(-v[1],   v[0],  0),
    )


def quat_to_rot(q):
    """Rotation matrix from unit quaternion q = [w, x, y, z]."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    return ca.vertcat(
        ca.horzcat(1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)),
        ca.horzcat(2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        ca.horzcat(2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)),
    )


def quat_mul(q1, q2):
    """Hamilton product q1 (x) q2, both [w, x, y, z]."""
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
    return ca.vertcat(
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


class LoadCableDynamics:
    """Symbolic CasADi model for n cable-suspended quadrotors carrying a load.

    Geometry/inertia (attachment points rho_i, cable lengths l_i, masses) are
    baked in as constants from the world; they are known and fixed per-world.
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

        self.nx = LOAD_DIM + CABLE_DIM * self.n
        self.nu = CTRL_DIM * self.n

        self.x = ca.MX.sym('x', self.nx)
        self.u = ca.MX.sym('u', self.nu)
        self._unpack()
        self.f_expl = self._build_f_expl()

    # ---- state/control accessors -------------------------------------------
    def _unpack(self):
        x = self.x
        self.p = x[0:3]
        self.v = x[3:6]
        self.q = x[6:10]
        self.w = x[10:13]
        self.R = quat_to_rot(self.q)
        self.s, self.r, self.rd, self.rdd, self.t, self.td = [], [], [], [], [], []
        for i in range(self.n):
            b = LOAD_DIM + CABLE_DIM * i
            self.s.append(x[b:b + 3])
            self.r.append(x[b + 3:b + 6])
            self.rd.append(x[b + 6:b + 9])
            self.rdd.append(x[b + 9:b + 12])
            self.t.append(x[b + 12])
            self.td.append(x[b + 13])
        self.gamma, self.lmbda = [], []
        for i in range(self.n):
            c = CTRL_DIM * i
            self.gamma.append(self.u[c:c + 3])
            self.lmbda.append(self.u[c + 3])

    # ---- load accelerations (shared by dynamics + thrust constraint) -------
    def _load_accel(self):
        v_dot = -sum((self.t[i] * self.s[i] for i in range(self.n)),
                     ca.MX.zeros(3)) / self.m + GRAVITY
        torque = sum((self.t[i] * ca.cross(self.R.T @ self.s[i], self.rho[i])
                      for i in range(self.n)), ca.MX.zeros(3))
        w_dot = self.Jinv @ (-ca.cross(self.w, self.J @ self.w) + torque)
        return v_dot, w_dot

    def _build_f_expl(self):
        v_dot, w_dot = self._load_accel()
        q_dot = 0.5 * quat_mul(self.q, ca.vertcat(0, self.w))
        rows = [self.v, v_dot, q_dot, w_dot]
        for i in range(self.n):
            rows += [
                ca.cross(self.r[i], self.s[i]),  # s_dot
                self.rd[i],                       # r_dot
                self.rdd[i],                      # rd_dot
                self.gamma[i],                    # rdd_dot
                self.td[i],                       # t_dot
                self.lmbda[i],                    # td_dot
            ]
        return ca.vertcat(*rows)

    # ---- quadrotor kinematics (Eq 5 and derivatives) -----------------------
    def quad_position(self, i):
        return self.p + self.R @ self.rho[i] - self.l[i] * self.s[i]

    def quad_velocity(self, i):
        # d/dt(R rho_i) = R (w x rho_i);  d/dt(s_i) = r_i x s_i
        return (self.v + self.R @ ca.cross(self.w, self.rho[i])
                - self.l[i] * ca.cross(self.r[i], self.s[i]))

    def quad_accel(self, i):
        v_dot, w_dot = self._load_accel()
        # d/dt[R(w x rho)] = R[ w_dot x rho + w x (w x rho) ]
        rot_term = self.R @ (ca.cross(w_dot, self.rho[i])
                             + ca.cross(self.w, ca.cross(self.w, self.rho[i])))
        # d/dt[r x s] = rd x s + r x (r x s)
        cab_term = (ca.cross(self.rd[i], self.s[i])
                    + ca.cross(self.r[i], ca.cross(self.r[i], self.s[i])))
        return v_dot + rot_term - self.l[i] * cab_term

    def thrust(self, i):
        """Collective thrust magnitude of drone i (Eq 9, drag neglected)."""
        f = self.mi[i] * (self.quad_accel(i) - GRAVITY) - self.t[i] * self.s[i]
        return ca.norm_2(f)


if __name__ == "__main__":
    # quick self-test for the three_soft geometry (3 drones at 120deg, R_a=0.08)
    n = 3
    rho = [[0.08 * np.cos(2 * np.pi * k / 3), 0.08 * np.sin(2 * np.pi * k / 3), 0.025]
           for k in range(n)]
    dyn = LoadCableDynamics(n, load_mass=0.4, load_inertia=[1.67e-3, 1.67e-3, 3.33e-3],
                            cable_lengths=[0.42] * n, attach_points=rho, drone_mass=0.6)
    print(f"n={n}  nx={dyn.nx} (expect {13 + 14 * n})  nu={dyn.nu} (expect {4 * n})")
    f = ca.Function('f', [dyn.x, dyn.u], [dyn.f_expl])
    x0 = np.zeros(dyn.nx)
    x0[6] = 1.0  # quat w = 1
    for i in range(n):
        x0[13 + 14 * i:13 + 14 * i + 3] = [0, 0, 1]   # s_i = +z (unit)
        x0[13 + 14 * i + 12] = 1.5                      # t_i = 1.5 N (taut)
    xdot = np.array(f(x0, np.zeros(dyn.nu))).flatten()
    print("f_expl ok, xdot[:6] (load p_dot,v_dot) =", np.round(xdot[:6], 3))
    Ti = ca.Function('T', [dyn.x, dyn.u], [dyn.thrust(0)])
    print("thrust(drone0) at rest =", float(Ti(x0, np.zeros(dyn.nu))), "N")
