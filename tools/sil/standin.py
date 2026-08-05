"""
tools/sil/standin.py
--------------------
The bench's stand-in approach controller, and the betaflight stick inverse it needs.

WHY THIS EXISTS. The SIL bench is also the "simple deterministic weld harness" that
decision D5 asks for, so that reconfiguration can be tested without waiting on the
collaborator's approach stack (risks R4/R10). Before the weld, something has to fly the
newcomer to its weld pose. That something is NOT the real tracker (which has no
reference until the network folds it in) and NOT the collaborator's approach MPC (out
of scope, and the point is to be independent of it).

It is deliberately a PLAIN controller and is clearly not part of the contribution:
cascade PD on position -> desired acceleration -> tilt attitude -> body rates ->
betaflight sticks. It flies the SAME 6-DOF plant as every other drone, so the
newcomer's state at the weld instant is a dynamically consistent state rather than a
teleport.

`betaflight_rates_inv` is the inverse of the rate curve. THESIS_PLAN §12.1 stage V
needs exactly this function ("stick = betaflight_rates_inv(w_cmd)  [NEW: invert
dynamics.py:123]"), so it is written once here and unit-tested against the forward
curve; when the velocity architecture is built it should be lifted into a package
rather than re-derived.
"""
import numpy as np

from .plant import G, betaflight_rates, quat_to_rot


def betaflight_rates_inv(rate_deg, d=70.0, f=670.0, g=0.5, tol=1e-9):
    """Stick deflection in [-1, 1] that produces `rate_deg` deg/s.

    The forward curve is continuous and strictly increasing in x, so a bisection is
    exact to tolerance and cannot fall into the local-minimum traps a Newton solve on
    the x^6 term can. 60 iterations on [-1, 1] is ~1e-18, i.e. limited by float64 and
    not by the loop.
    """
    lo, hi = -1.0, 1.0
    r_lo = betaflight_rates(lo, d, f, g)
    r_hi = betaflight_rates(hi, d, f, g)
    if rate_deg <= r_lo:
        return -1.0
    if rate_deg >= r_hi:
        return 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if betaflight_rates(mid, d, f, g) < rate_deg:
            lo = mid
        else:
            hi = mid
        if hi - lo < tol:
            break
    return 0.5 * (lo + hi)


def tilt_quat_from_accel(a_des, heading=0.0):
    """Level-to-thrust-direction quaternion: the shortest rotation taking body +z onto
    the desired acceleration direction, composed with a yaw of `heading`.

    Same construction as acados._tilt_quat_from_accel -- a yaw-free tilt, so a drone
    never spins to align a heading it was not asked to hold."""
    a = np.asarray(a_des, float)
    n = float(np.linalg.norm(a))
    if n < 1e-9:
        zb = np.array([0.0, 0.0, 1.0])
    else:
        zb = a / n
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(z, zb)
    c = float(np.dot(z, zb))
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        q_tilt = np.array([1.0, 0.0, 0.0, 0.0]) if c > 0 else np.array([0.0, 1.0, 0.0, 0.0])
    else:
        ang = np.arctan2(s, c)
        axis = v / s
        q_tilt = np.concatenate([[np.cos(ang / 2)], np.sin(ang / 2) * axis])
    q_yaw = np.array([np.cos(heading / 2), 0.0, 0.0, np.sin(heading / 2)])
    w0, x0, y0, z0 = q_yaw
    w1, x1, y1, z1 = q_tilt
    return np.array([
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ])


class ApproachStandin:
    """Holds one drone at a commanded world position. Emits ELRS channel values.

    kp/kd are deliberately modest: this is a stand-in for a real approach controller,
    not a superhuman one, so the newcomer arrives at the weld with realistic residual
    error and velocity rather than pinned to a point.
    """

    def __init__(self, thrust_c=88.6, kp=6.0, kd=4.0, k_att=8.0,
                 rates=(70.0, 670.0, 0.5), max_tilt_deg=25.0):
        self.thrust_c = float(thrust_c)
        self.kp, self.kd, self.k_att = float(kp), float(kd), float(k_att)
        self.rates = rates
        self.max_tilt = np.radians(max_tilt_deg)

    def channels(self, p, v, q, p_ref, v_ref=None):
        """Return (channel_0, channel_1, channel_2, channel_3) for one cycle."""
        v_ref = np.zeros(3) if v_ref is None else np.asarray(v_ref, float)
        a_des = (self.kp * (np.asarray(p_ref, float) - np.asarray(p, float))
                 + self.kd * (v_ref - np.asarray(v, float))
                 + np.array([0.0, 0.0, G]))
        # limit the commanded tilt so the stand-in cannot ask for an attitude no
        # quadrotor would hold, which would make the pre-weld approach unrealistic
        horiz = float(np.linalg.norm(a_des[:2]))
        max_h = max(a_des[2], 1e-3) * np.tan(self.max_tilt)
        if horiz > max_h:
            a_des[:2] *= max_h / horiz
        thrust = float(np.linalg.norm(a_des))
        throttle = float(np.clip(np.sqrt(max(thrust, 0.0) / self.thrust_c), 0.0, 1.0))

        q_des = tilt_quat_from_accel(a_des)
        # body-frame attitude error -> proportional rate command
        R = quat_to_rot(q)
        Rd = quat_to_rot(q_des)
        Re = R.T @ Rd
        # log map of a near-identity rotation (small-angle vee), body frame
        e = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0],
                            Re[1, 0] - Re[0, 1]])
        w_cmd = np.degrees(self.k_att * e)
        d, f, g = self.rates
        ch0 = betaflight_rates_inv(float(w_cmd[0]), d, f, g)
        ch1 = betaflight_rates_inv(float(w_cmd[1]), d, f, g)
        ch3 = -betaflight_rates_inv(float(w_cmd[2]), d, f, g)   # plant negates ch3
        return ch0, ch1, throttle * 2.0 - 1.0, ch3
