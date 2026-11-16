"""disturbance_estimator.py -- constant external forces on the load and on every drone, from
measured motion and delivered thrust (card docs/experiments/2026-10-03_offset_free_mpc.md).

Per sample the force balance of each drone and of the load is solved by least squares for the
n rod tensions t_i, one force d_D shared by every drone and one force d_L on the load:

    drone i:  t_i s_i + d_D = m_d a_i - f_i - m_d g - (rod weight handed to the drone end)
    load:    -sum t_i s_i + d_L = m_L a_L - m_L g - sum (rod weight handed to the load end)

s_i is the unit rod direction from the drone's pivot to its plate, f_i the thrust the tracker
delivered (its own map) along the drone's body z, t_i the tension at the drone end. A rod of mass
m_r pinned at both ends hands its weight to the ends: the part across the rod by the lever (lam,
the centre of mass's fraction of the rod length from the load end, at the drone end), the part
along it with the tension. Nothing comes from the plan (the 09-23 estimator converged to the
plan's own acceleration). Pure: no ROS, so it is unit-tested and replayable offline.
"""
import numpy as np

G = np.array([0.0, 0.0, -9.81])


def rod_end_loads(s, m_r, lam):
    """(on the drone, on the load) of one rod's weight, tension excluded; s points drone -> load."""
    e = -np.asarray(s, float)
    g_ax = m_r * float(G @ e) * e
    return lam * (m_r * G - g_ax), lam * g_ax + (1.0 - lam) * m_r * G


def solve_split(s_dirs, thrust, a_drones, a_load, m_d, m_load, m_r=0.0, lam=0.0):
    """Least-squares (d_L, d_D, tensions, condition number) of one sample. thrust: force vectors."""
    n = len(s_dirs)
    A = np.zeros((3 * n + 3, n + 6))
    b = np.zeros(3 * n + 3)
    for i in range(n):
        s = np.asarray(s_dirs[i], float)
        on_drone, on_load = rod_end_loads(s, m_r, lam)
        A[3 * i:3 * i + 3, i] = s
        A[3 * i:3 * i + 3, n:n + 3] = np.eye(3)
        b[3 * i:3 * i + 3] = (m_d * np.asarray(a_drones[i], float) - np.asarray(thrust[i], float)
                              - m_d * G - on_drone)
        A[3 * n:, i] = -s
        b[3 * n:] -= on_load
    A[3 * n:, n + 3:] = np.eye(3)
    b[3 * n:] += m_load * np.asarray(a_load, float) - m_load * G
    x = np.linalg.lstsq(A, b, rcond=None)[0]
    return x[n + 3:], x[n:n + 3], x[:n], float(np.linalg.cond(A))


class DisturbanceEstimator:
    """Low-passed d_L, d_D. Accelerations from successive measured velocities; the same filter
    then acts on every term of the balance: two first-order stages of time constant tau each.
    Offline on the logged raw estimate (R0862, R0864) two stages of 0.6 s halve the noise of one
    stage of 1.0 s at the same 2.3 s to 90 % of a step. Frozen while not gated."""

    def __init__(self, n, tau=0.6, bound_load=4.0, bound_drone=1.5):
        self.tau = float(tau)
        self.bound_load = float(bound_load)
        self.bound_drone = float(bound_drone)
        self.reset(n)

    def reset(self, n=None):
        if n is not None:
            self.n = int(n)
        self.d_L = np.zeros(3)
        self.d_D = np.zeros(3)
        self._s1 = (np.zeros(3), np.zeros(3))   # first filter stage
        self.raw = None                  # last unfiltered (d_L, d_D, tensions, cond)
        self.railed = False
        self._prev = None                # (t, drone velocities, load velocity)

    def update(self, t, gated, s_dirs, thrust, v_drones, v_load, m_d, m_load, m_r=0.0, lam=0.0):
        """One tick at time t (s). Returns (d_L, d_D)."""
        prev, self._prev = self._prev, (float(t), [np.asarray(v, float) for v in v_drones],
                                        np.asarray(v_load, float))
        if prev is None or not gated:
            return self.d_L, self.d_D
        dt = float(t) - prev[0]
        if dt <= 0.0:
            return self.d_L, self.d_D
        a_dr = [(v - v0) / dt for v, v0 in zip(self._prev[1], prev[1])]
        a_load = (self._prev[2] - prev[2]) / dt
        self.raw = solve_split(s_dirs, thrust, a_dr, a_load, m_d, m_load, m_r, lam)
        k = dt / (self.tau + dt)
        bL, bD = self.bound_load, self.bound_drone
        s1L = np.clip(self._s1[0] + k * (self.raw[0] - self._s1[0]), -bL, bL)
        s1D = np.clip(self._s1[1] + k * (self.raw[1] - self._s1[1]), -bD, bD)
        self._s1 = (s1L, s1D)
        d_L = self.d_L + k * (s1L - self.d_L)
        d_D = self.d_D + k * (s1D - self.d_D)
        self.d_L = np.clip(d_L, -bL, bL)
        self.d_D = np.clip(d_D, -bD, bD)
        self.railed = bool(np.any(np.abs(s1L) >= bL) or np.any(np.abs(s1D) >= bD))
        return self.d_L, self.d_D
