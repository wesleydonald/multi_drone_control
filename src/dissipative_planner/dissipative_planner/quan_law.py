"""
quan_law.py
-----------
The per-robot dissipative force law of Quan et al., "Self-Organizing Aerial Swarm Robotics: A Table-Mechanics-Inspired
Approach" (arXiv 2509.03563), as its text prints it: Eq. 3 (the virtual node), Eq. 5 (the perception range), Eq. 7-9
(the force) and Eq. 10 (the acceleration command), p.24-25, with the supplementary's S2 (the same law, Eq. control1, and
the node dynamics) where the main text leaves a point open. The thesis's "Quan et al. exactly" arm (decisions
2026-10-23; card docs/experiments/2026-10-23_quan_exact.md). It is not our port (dissipative_network.py, which follows
the authors' node-network code).

Each robot i computes its command from what it perceives (Fig. 2, p.9): its own position and velocity, those of the
robots within range r (Eq. 5), and the load's. Pure numpy, no ROS.

    q_i   = x_i + (h_c - h_i) / (h_i - h_L) (x_i - x_L)                                       (Eq. 3)
    u_i   = h1_i + h2_i - f_i - m_i g                                                          (Eq. 7)
    h1_i  = - k_d,i k_ij  sum_j w_ij ((1 - l_ij(0)/l_ij) (q_i - q_j))_hor
            + k_i (1 - l_i(0)/l_i) (q_i - x_i)
            - k_d,i c_ij  sum_j (w_ij/l_ij^2 (q_i - q_j)^T (q'_i - q'_j) (q_i - q_j))_hor
            + c_i/l_i^2 (q_i - x_i)^T (q'_i - x'_i) (q_i - x_i)                                (Eq. 8)
    h2_i  = - k_d,i f_c (q'_i - x'_0)_hor                                                      (Eq. 9)
    a_cmd = a_expected + u_i/m_i + f_i/m_i + g  =  a_expected + (h1_i + h2_i)/m_i              (Eq. 10)
    k_d,i = (h_L - h_i) / (h_L - h_c),  j = 0..n (j = 0 the formation centre q_0 = x_0), w_ij = 1 within r

READINGS: what the text leaves open, each settled here and nowhere else, with its source (the card's readings table;
each one is stated in the thesis):
  R1 the node (QuanParams.nodes): 'algebraic' = Eq. 3 of the main text, q'_i its time derivative from the measured
     x'_i and x'_L; 'dynamic' = S2/S4: each node a point mass (node_mass, init.m's 1 kg) on the plane h_c, seeded at
     Eq. 3's node at the hand-over and moved by the forces of S4's Lyapunov function (the node-node and node-centre
     springs and dampers, the friction, the robot-node spring and damper's reaction), the robot flying Eq. 7 against
     it. advance_nodes().
  R2 the k_d,i bracket (Eq. 7 as typeset, p.25; S2 Eq. control1): k_d,i multiplies the node-node sums and the friction
     only; the robot-node spring k_i and damper c_i stand outside it with a + sign. forces().
  R3 x_0 = q_0, "the formation center": the commanded path point on the node plane h_c, x'_0 the path's velocity, a
     rigid leader (S2: "the center ... can be a specific robot ... leader-follower"; init.m's desired centre p00 on the
     node plane). w_i0 = 1.
  R4 h_c, "the pre-designed altitude": the node plane hc_m above the load's commanded height, hc_m = 3.41 cable lengths
     (init.m: each robot placed 1.0 m from the payload, mass + r/rr*r0 with r = rr = 2.02; node plane h0 -12 against
     the payload's -8.5858: 3.41 m; 2.02 is the node-node rest length L0). It moves with the commanded height.
  R5 a_expected,i: the path's commanded acceleration (zero in hover).
  R6 f_i and g in Eq. 10 cancel (S2: m x'' = m g + f_i + u_i), so a_cmd = a_expected + (h1 + h2)/m_i. The thrust
     feed-forward of the stand-in autopilot carries the weight and a modelled pull, (m_L/n_i + rod_mass) g / sin phi_i
     along the measured load-to-robot line (n_i the robots perceived, itself included; gated by the rod's tautness).
     The paper has no such model (their robots cancel the true tension in S2); it is the stand-in's. cable_pull().
     The masses are the planner's: drone_mass already carries the rod in the twin (0.475 kg airframe + 0.075 kg rod
     and magnet), so rod_mass is only for a rod outside drone_mass.
  R6b what the model misses of f_i, vertically (QuanParams.fz): 'model' (R6 alone, the default); 'autopilot' adds the
     integral the paper's autopilot runs on its velocity error (S1: PX4 "executes velocity and acceleration
     commands"; Eq. 10 sends no f_i or g, so the autopilot carries them), vertical only: b_i += fz_ki (v_d,z of the
     last tick - x'_i,z) dt, within fz_max, added to the thrust feed-forward. In a still hover it settles where
     a_cmd,z = 0, the paper's equilibrium with the true f_i cancelled (S2: m x'' = m g + f_i + u_i), so a model error
     no longer rests on the robot-node spring k_i. The integral starts where the drone's thrust already was: an
     autopilot keeps its hover thrust across a mode change, so b_i(0) makes the first vertical feed-forward the one
     flown before the hand-over (start(a_cable0=...): our OCP's cable term, settled on the ring).
  R7 acceleration -> velocity command (QuanParams.vcmd): 'measured' (R7b, the default) v_d = x'_i + a_cmd dt, no
     accumulated state, with a_cmd also the acceleration feed-forward (S1: PX4 "executes velocity and acceleration
     commands"; Fig. 2 feeds v_self to the controller); 'integrated' (R7, kept for the comparison) v_d integrates
     a_cmd from the hand-over's velocity. Both held within v_max.
  R8 gains and range: r = 2 m (their LiDAR filter, p.22): every robot sees every other at 1 m cables; the
     coefficients from the card's offline search, started from init.m's (k 10 per edge, c 10, node mass 1 kg).
     l_ij(0), l_i(0) at the hand-over.
  R9 rate: the law at the planner's 10 Hz (their simulation: 100 Hz, init.m step 0.01); dynamic nodes integrated in
     10 substeps a tick.
  R10 the load is a point (Eq. 1; S2's p_{n+2}): its attitude is never read, so the formation has no frame.
"""
import numpy as np

G = 9.81
# Eq. 3 is singular with a robot level with the load (h_i = h_L); below this the robot is treated as level
MIN_DH = 0.05
NODE_SUBSTEPS = 10


class QuanParams:
    """The law's coefficients (R8) and its readings (R1, R4, R6, R7)."""
    def __init__(self, k_ij=5.0, c_ij=5.0, k_i=5.0, c_i=5.0, f_c=2.5, hc_m=3.41, range_m=2.0, v_max=2.0,
                 nodes='algebraic', vcmd='measured', node_mass=1.0, rod_mass=0.0, fz='model', fz_ki=2.0, fz_max=3.0):
        self.k_ij = float(k_ij)       # N/m, node-node and node-centre springs
        self.c_ij = float(c_ij)       # N s/m, node-node and node-centre dampers
        self.k_i = float(k_i)         # N/m, robot-node spring
        self.c_i = float(c_i)         # N s/m, robot-node damper
        self.f_c = float(f_c)         # N s/m, friction to the formation centre (Eq. 9)
        self.hc_m = float(hc_m)       # m, node plane above the load's commanded height (R4)
        self.range_m = float(range_m)  # m, perception range r (Eq. 5)
        self.v_max = float(v_max)     # m/s, |v_d| bound (R7)
        self.nodes = str(nodes)       # R1: 'algebraic' | 'dynamic'
        self.vcmd = str(vcmd)         # R7: 'measured' (R7b) | 'integrated' (R7)
        self.node_mass = float(node_mass)  # kg, a dynamic node's mass (R1)
        self.rod_mass = float(rod_mass)    # kg per rod in the pull model (R6)
        self.fz = str(fz)             # R6b: 'model' | 'autopilot'
        self.fz_ki = float(fz_ki)     # 1/s^2, R6b's vertical integral gain
        self.fz_max = float(fz_max)   # m/s^2, its bound
        if self.nodes not in ('algebraic', 'dynamic') or self.vcmd not in ('measured', 'integrated') \
                or self.fz not in ('model', 'autopilot'):
            raise ValueError(f'quan nodes {self.nodes!r} / vcmd {self.vcmd!r} / fz {self.fz!r}')


def hor(v):
    """The vector with its z component zero (the paper's subscript "hor")."""
    v = np.array(v, float)
    v[..., 2] = 0.0
    return v


def _dh(h_i, h_L):
    return max(float(h_i) - float(h_L), MIN_DH)


def node(x_i, x_L, h_c):
    """Eq. 3: robot i's virtual node, on the load-robot line at altitude h_c."""
    x_i, x_L = np.asarray(x_i, float), np.asarray(x_L, float)
    return x_i + (float(h_c) - x_i[2]) / _dh(x_i[2], x_L[2]) * (x_i - x_L)


def node_rate(x_i, xd_i, x_L, xd_L, h_c, hc_dot=0.0):
    """R1 algebraic: the time derivative of Eq. 3 from the measured velocities (z = hc_dot: the node stays on its
    plane)."""
    x_i, xd_i = np.asarray(x_i, float), np.asarray(xd_i, float)
    x_L, xd_L = np.asarray(x_L, float), np.asarray(xd_L, float)
    dh = _dh(x_i[2], x_L[2])
    s = (float(h_c) - x_i[2]) / dh
    s_dot = (float(hc_dot) - xd_i[2]) / dh - (float(h_c) - x_i[2]) * (xd_i[2] - xd_L[2]) / dh ** 2
    return xd_i + s_dot * (x_i - x_L) + s * (xd_i - xd_L)


def k_d(h_i, h_L, h_c):
    """Eq. 7's k_d,i = (h_L - h_i) / (h_L - h_c)."""
    return _dh(h_i, h_L) / max(float(h_c) - float(h_L), MIN_DH)


class QuanLaw:
    """The law for a fleet of n robots, one command per robot from its own view. start() at the hand-over records the
    rest lengths (and seeds dynamic nodes); each tick advance_nodes() once, then command() per robot."""

    def __init__(self, n, drone_mass, load_mass, params=None, g=G):
        self.n = int(n)
        self.m = float(drone_mass)
        self.m_L = float(load_mass)
        self.g = float(g)
        self.p = params or QuanParams()
        self.l0 = np.zeros((self.n, self.n + 1))   # l_ij(0), column 0 the formation centre
        self.li0 = np.zeros(self.n)                # l_i(0)
        self.v_d = np.zeros((self.n, 3))
        self.b_z = np.zeros(self.n)                # R6b's vertical integral, m/s^2
        self.q = np.zeros((self.n, 3))             # dynamic nodes (R1)
        self.qd = np.zeros((self.n, 3))
        self.started = False

    def start(self, x, xd, x_L, x_0, h_c, xd_L=None, a_cable0=None):
        """The hand-over: l_ij(0) and l_i(0) from the measured state (R8), each v_d at the robot's velocity (R7),
        dynamic nodes at Eq. 3's nodes and their rates (R1); with R6b, each b_i(0) from a_cable0 (n x 3, m/s^2), the
        cable term each drone's thrust carried before it (rows of NaN or None: b_i(0) = 0)."""
        x, xd = np.asarray(x, float), np.asarray(xd, float)
        xd_L = np.zeros(3) if xd_L is None else np.asarray(xd_L, float)
        self.q = np.array([node(x[i], x_L, h_c) for i in range(self.n)])
        self.qd = hor(np.array([node_rate(x[i], xd[i], x_L, xd_L, h_c) for i in range(self.n)]))
        q0 = self._centre(x_0, h_c)
        for i in range(self.n):
            self.l0[i, 0] = float(np.linalg.norm(self.q[i] - q0))
            for j in range(self.n):
                self.l0[i, j + 1] = float(np.linalg.norm(self.q[i] - self.q[j]))
            self.li0[i] = float(np.linalg.norm(self.q[i] - x[i]))
        self.v_d = xd.copy()
        self.b_z = np.zeros(self.n)
        if self.p.fz == 'autopilot' and a_cable0 is not None:
            for i in range(self.n):
                ac0 = a_cable0[i]
                if ac0 is not None and np.all(np.isfinite(ac0)):
                    b = float(self.cable_pull(i, x, x_L)[2]) / self.m - float(ac0[2])
                    self.b_z[i] = float(np.clip(b, -self.p.fz_max, self.p.fz_max))
        self.started = True

    @staticmethod
    def _centre(x_0, h_c):
        return np.array([float(x_0[0]), float(x_0[1]), float(h_c)])

    def neighbours(self, i, x):
        """Eq. 5: the robots j != i within range r of robot i (w_ij = 1)."""
        return [j for j in range(self.n) if j != i
                and float(np.linalg.norm(np.asarray(x[j], float) - np.asarray(x[i], float))) < self.p.range_m]

    def nodes_now(self, x, xd, x_L, xd_L, h_c, hc_dot=0.0):
        """Every node and its velocity under the R1 reading."""
        if self.p.nodes == 'dynamic':
            return self.q, self.qd
        q = np.array([node(x[i], x_L, h_c) for i in range(self.n)])
        qd = np.array([node_rate(x[i], xd[i], x_L, xd_L, h_c, hc_dot) for i in range(self.n)])
        return q, qd

    def _node_terms(self, i, x, q, qd, x_0, xd_0, h_c, hc_dot):
        """The node-node and node-centre spring and damper sums of node i (before k_d), unprojected."""
        others = [(self._centre(x_0, h_c), np.array([float(xd_0[0]), float(xd_0[1]), float(hc_dot)]),
                   self.l0[i, 0])]
        others += [(q[j], qd[j], self.l0[i, j + 1]) for j in self.neighbours(i, x)]
        spring, damp = np.zeros(3), np.zeros(3)
        for q_j, qd_j, l0 in others:
            d = q[i] - q_j
            l = max(float(np.linalg.norm(d)), 1e-6)
            spring += (1.0 - l0 / l) * d
            damp += float(d @ (qd[i] - qd_j)) / l ** 2 * d
        return spring, damp

    def _robot_node_terms(self, i, x, xd, q, qd):
        """The robot-node spring and damper on robot i (Eq. 8's k_i and c_i terms)."""
        e = q[i] - np.asarray(x[i], float)
        li = max(float(np.linalg.norm(e)), 1e-6)
        return (self.p.k_i * (1.0 - self.li0[i] / li) * e
                + self.p.c_i / li ** 2 * float(e @ (qd[i] - np.asarray(xd[i], float))) * e)

    def forces(self, i, x, xd, x_L, xd_L, x_0, xd_0, h_c, hc_dot=0.0):
        """(h1_i, h2_i) of Eq. 8 and 9 (N) for robot i from its view: x, xd every robot's position and velocity (only
        those in range are used), the load's, the formation centre's (R3) and the node plane (R4)."""
        x, xd = np.asarray(x, float), np.asarray(xd, float)
        q, qd = self.nodes_now(x, xd, x_L, xd_L, h_c, hc_dot)
        spring, damp = self._node_terms(i, x, q, qd, x_0, xd_0, h_c, hc_dot)
        kd = k_d(x[i][2], x_L[2], h_c)
        h1 = -kd * self.p.k_ij * hor(spring) - kd * self.p.c_ij * hor(damp) + self._robot_node_terms(i, x, xd, q, qd)
        h2 = -kd * self.p.f_c * hor(qd[i] - np.array([float(xd_0[0]), float(xd_0[1]), 0.0]))
        return h1, h2

    def advance_nodes(self, x, xd, x_0, xd_0, h_c, dt, hc_dot=0.0):
        """R1 dynamic: one tick of the nodes' own dynamics (S4's Lyapunov function: the springs and dampers among the
        nodes and to the centre, the friction, the robots' springs and dampers on them), on the plane h_c. A no-op for
        algebraic nodes."""
        if self.p.nodes != 'dynamic':
            return
        x, xd = np.asarray(x, float), np.asarray(xd, float)
        h = float(dt) / NODE_SUBSTEPS
        for _ in range(NODE_SUBSTEPS):
            f = np.zeros((self.n, 3))
            for i in range(self.n):
                spring, damp = self._node_terms(i, x, self.q, self.qd, x_0, xd_0, h_c, hc_dot)
                f[i] = -hor(self.p.k_ij * spring + self.p.c_ij * damp
                            + self.p.f_c * (self.qd[i] - np.array([float(xd_0[0]), float(xd_0[1]), 0.0]))
                            + self._robot_node_terms(i, x, xd, self.q, self.qd))
            self.qd = self.qd + f / self.p.node_mass * h
            self.q = self.q + self.qd * h
        self.q[:, 2] = float(h_c)
        self.qd[:, 2] = float(hc_dot)

    def cable_pull(self, i, x, x_L, taut_gate=1.0):
        """R6: f_i, the cable's modelled pull on robot i (N): its share of the load and its rod's weight along the
        measured load-to-robot line, toward the load."""
        u = np.asarray(x[i], float) - np.asarray(x_L, float)
        u = u / max(float(np.linalg.norm(u)), 1e-6)
        n_i = 1 + len(self.neighbours(i, x))
        t = (self.m_L / n_i + self.p.rod_mass) * self.g / max(float(u[2]), 0.05)
        return -float(taut_gate) * t * u

    def command(self, i, x, xd, x_L, xd_L, x_0, xd_0, a_exp, h_c, dt, hc_dot=0.0, taut_gate=1.0):
        """Robot i's command for this tick: (v_d, a_ff, a_cable, a_cmd), the velocity command, the specific-thrust
        feed-forward, the cable pull per kg and Eq. 10's acceleration (m/s, m/s^2), in the network's wire convention
        a_ff = g + a_cmd - a_cable (R6, R7), plus R6b's vertical integral."""
        h1, h2 = self.forces(i, x, xd, x_L, xd_L, x_0, xd_0, h_c, hc_dot)
        a_cmd = np.asarray(a_exp, float) + (h1 + h2) / self.m          # Eq. 10 with f_i and g cancelled (R6)
        a_cable = self.cable_pull(i, x, x_L, taut_gate) / self.m
        if self.p.fz == 'autopilot':
            e = float(self.v_d[i][2]) - float(np.asarray(xd[i], float)[2])     # last tick's command, met or not
            self.b_z[i] = float(np.clip(self.b_z[i] + self.p.fz_ki * e * float(dt), -self.p.fz_max, self.p.fz_max))
        base = self.v_d[i] if self.p.vcmd == 'integrated' else np.asarray(xd[i], float)
        v = base + a_cmd * float(dt)
        s = float(np.linalg.norm(v))
        self.v_d[i] = v if s <= self.p.v_max else v * (self.p.v_max / s)
        a_ff = np.array([0.0, 0.0, self.g + self.b_z[i]]) + a_cmd - a_cable
        return self.v_d[i].copy(), a_ff, a_cable, a_cmd
