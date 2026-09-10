"""
velocity_loop.py — the paper's architecture: network reference -> velocity -> rates.

Design note: docs/design/velocity_loop.md. Read §7 (what would make this the wrong
answer) before quoting a comparison from it.

Quan et al.'s Fig. 2 sends the dissipative controller's output as a VELOCITY COMMAND
straight to the autopilot. This repo instead makes it a position setpoint for a 2 s
horizon MPC, and that inserted stage is where the tracking error measurably lives
(DISSIPATIVE_TRACKING_ISSUE.md §2; re-measured on R0054, tracker stage 0.869 radius
ratio and +0.538 s of the +0.684 s total lag). This class is the missing layer: their
autopilot accepts velocity natively, Betaflight accepts body rates and throttle, so the
velocity -> attitude -> rate conversion theirs does internally is ours to supply.

PURE: no ROS, no acados, no I/O. That is what lets the SIL bench and the unit tests
drive it directly, and it is architecture principle 2.

The loop, per drone per tick:

    v_sp  = v_ref + kp_pos * (p_ref - p)          clamped to v_max
    e_v   = v_sp - v
    I    += e_v * dt                              clamped so |ki*I| <= a_i_max
    a_sp  = a_ff + kv * e_v + ki * I
    thr, q_sp = tilt_quat_from_accel(a_sp, kT)
    w_cmd = k_att * quat_error(q_sp, q)           clamped to w_max
    stick = rates_inv(w_cmd)

`a_ff` is the planner's required SPECIFIC THRUST acceleration and already contains
gravity and cable tension, so the feedback terms are corrections on top of it and hover
needs no separate gravity term.

MEASURED-FORCE THRUST (indi_gain > 0, docs/experimentation/control_methods_survey.md R1):
the throttle magnitude becomes INCREMENTAL on the IMU. With b the body thrust axis,

    xdd_des  = a_sp - g*z + a_cable            (what the loop wants the drone to DO)
    xdd_meas = R f_imu - g*z                   (what it is doing; f_imu = thrust+cable)
    T_next   = T_applied_filtered + clip((xdd_des - xdd_meas) . b, +-indi_a_max)
    throttle = throttle_nominal + indi_gain * (T_next / kT - throttle_nominal)

so a wrong kT, a wrong load mass or a wrong cable model shows up as a transient the
increment removes, not as a steady position offset -- the mechanism Sun et al.'s INDI
low level and the tension-to-thrust feed-forward of arXiv 2605.05339 rely on. The
applied throttle and the IMU pass through the SAME first-order filter (indi_tau); that
synchronisation is what keeps the increment from fighting actuator lag.
"""
import math

import numpy as np

# Betaflight rate curve constants, matching dynamics.py's model parameters (the values
# controller_mpc.py passes as est_params). Kept here as defaults only -- the caller
# passes whatever the drone is actually configured with.
CENTRE_RATE_DEG = 70.0
MAX_RATE_DEG = 670.0
RATE_EXPO = 0.5


def betaflight_rates(stick, centre=CENTRE_RATE_DEG, max_rate=MAX_RATE_DEG,
                     expo=RATE_EXPO):
    """Stick in [-1, 1] -> commanded body rate in deg/s. Mirrors dynamics.py:123."""
    a = abs(float(stick))
    h = expo * a ** 6 + (1.0 - expo) * a ** 2
    return math.copysign(centre * a + (max_rate - centre) * h, stick)


def betaflight_rates_inv(rate_deg, centre=CENTRE_RATE_DEG, max_rate=MAX_RATE_DEG,
                         expo=RATE_EXPO, tol=1e-12):
    """Commanded body rate in deg/s -> stick in [-1, 1]. Inverts `betaflight_rates`.

    Bisection, not Newton: the forward curve is a sextic with no closed-form inverse,
    it is strictly increasing on [0, 1] for max_rate >= centre >= 0 and expo in [0, 1],
    and bisection on a monotone function cannot diverge. ~40 iterations for 1e-12, which
    at 50 Hz is free -- and this runs in the flight path, where a Newton step that walks
    off near the flat part of the curve would be a rate spike.

    Rates beyond the curve's range saturate at full stick rather than extrapolating."""
    target = abs(float(rate_deg))
    full = centre + (max_rate - centre)          # = max_rate, i.e. j(1)
    if target >= full:
        return math.copysign(1.0, rate_deg)
    lo, hi = 0.0, 1.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if hi - lo < tol:
            break
        if centre * mid + (max_rate - centre) * (expo * mid ** 6
                                                 + (1.0 - expo) * mid ** 2) < target:
            lo = mid
        else:
            hi = mid
    return math.copysign(0.5 * (lo + hi), rate_deg)


def _quat_to_rot(q):
    """wxyz quaternion -> rotation matrix (body -> world)."""
    w, x, y, z = _normalize(np.asarray(q, float))
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _normalize(q):
    n = float(np.linalg.norm(q))
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def tilt_quat_from_accel(a_sp, thrust_ratio, heading=0.0, thr_lo=0.05, thr_hi=0.6):
    """Required specific thrust acceleration (world) -> (throttle, quaternion wxyz).

    Same construction as acados.py's `_tilt_quat_from_accel`, reimplemented here rather
    than imported so this module stays free of the acados import chain -- the SIL bench
    and the unit tests must be able to use it without a compiled solver present. The
    round-trip against that function is pinned by a test."""
    a = np.asarray(a_sp, float)
    q_head = np.array([math.cos(0.5 * heading), 0.0, 0.0, math.sin(0.5 * heading)])
    nrm = float(np.linalg.norm(a))
    if nrm < 1e-3:                       # slack cable / freefall: hover, level
        return 9.81 / thrust_ratio, q_head
    throttle = float(np.clip(nrm / thrust_ratio, thr_lo, thr_hi))
    ax, ay, az = a / nrm
    q_tilt = _normalize(np.array([1.0 + az, -ay, ax, 0.0]))
    w0, x0, y0, z0 = q_tilt
    w1, x1, y1, z1 = q_head
    q = np.array([w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
                  w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
                  w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
                  w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1])
    return throttle, _normalize(q)


def limit_tilt(a_sp, tilt_max_deg, a_z_min=1.0):
    """Limit how far the commanded thrust vector may tilt from vertical.

    Every velocity-control autopilot has this and ours was missing it, which is the
    layer we are standing in for. Without it the horizontal term is unbounded while the
    vertical one is not: throttle clamps at 0.6, so a drone that cannot reach its
    reference keeps adding lateral acceleration against a saturated vertical one and
    the tilt runs away. Measured on the bench (R0067) during the lift -- 21 -> 33 -> 69
    deg with the tracking error still under 0.1 m, until the envelope check disarmed
    the fleet.

    The HORIZONTAL part is scaled and the vertical part is kept: losing altitude to
    chase a lateral error is how a carrying fleet drops its load."""
    a = np.asarray(a_sp, float).copy()
    az = max(float(a[2]), a_z_min)      # never command an inverted or zero-lift thrust
    h = a[:2]
    h_norm = float(np.linalg.norm(h))
    h_max = az * math.tan(math.radians(tilt_max_deg))
    if h_norm > h_max > 0.0:
        a[:2] = h * (h_max / h_norm)
    a[2] = az
    return a


def attitude_error(q_sp, q_meas):
    """Body-frame rotation vector (rad) taking `q_meas` onto `q_sp`.

    The SHORTEST rotation: a quaternion and its negation are the same attitude, so the
    error is taken against whichever sign has a non-negative scalar part. Without that,
    an attitude 1 degree away on the far side of the double cover commands a 359 degree
    slew, which on a drone carrying a load is a flip."""
    qs, qm = _normalize(np.asarray(q_sp, float)), _normalize(np.asarray(q_meas, float))
    w0, x0, y0, z0 = qm
    w1, x1, y1, z1 = qs
    # q_err = conj(q_meas) (x) q_sp
    q_err = np.array([w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1,
                      w0 * x1 - x0 * w1 - y0 * z1 + z0 * y1,
                      w0 * y1 + x0 * z1 - y0 * w1 - z0 * x1,
                      w0 * z1 - x0 * y1 + y0 * x1 - z0 * w1])
    if q_err[0] < 0.0:
        q_err = -q_err
    v = q_err[1:]
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        return np.zeros(3)
    return v / s * (2.0 * math.atan2(s, float(q_err[0])))


class VelocityLoop:
    """Network reference + measured state -> (roll, pitch, throttle, yaw) sticks.

    One instance per drone. `reset()` on every arm; `step()` at the control rate.
    """

    def __init__(self, kp_pos=2.0, kv=4.0, ki=1.0, k_att=8.0,
                 v_max=2.0, a_i_max=2.0, w_max_deg=300.0, yaw_k=2.0,
                 tilt_max_deg=30.0,
                 centre_rate_deg=CENTRE_RATE_DEG, max_rate_deg=MAX_RATE_DEG,
                 rate_expo=RATE_EXPO,
                 indi_gain=0.0, indi_tau=0.05, indi_a_max=3.0, indi_slope_ratio=1.0,
                 indi_thr_min=0.2):
        # floor on the INDI-modified throttle. The increment will cut thrust to whatever
        # makes the measured acceleration match the command -- on a newcomer whose rigid
        # rod momentarily PUSHES, that is ~zero, and a quadrotor at zero throttle has no
        # rate authority (Gazebo R0242: drone 3 to the 0.05 clip, flipped 2 s later).
        # Hover is ~0.39 here; 0.2 keeps half the attitude authority in the worst case.
        self.indi_thr_min = float(indi_thr_min)
        self.indi_gain = float(np.clip(indi_gain, 0.0, 1.0))
        self.indi_tau = float(indi_tau)
        self.indi_a_max = float(indi_a_max)
        # control effectiveness da/du divided by kT. The increment must use the LOCAL
        # slope of the thrust curve, not the secant the nominal throttle uses: for the
        # sim's a = c*u^2 the tangent is exactly 2x the secant at any operating point, so
        # a ratio of 1 doubles the increment loop gain (SIL R0241). 1.0 = linear curve.
        self.indi_slope_ratio = max(float(indi_slope_ratio), 0.1)
        self.kp_pos = float(kp_pos)
        self.kv = float(kv)
        self.ki = float(ki)
        self.k_att = float(k_att)
        self.v_max = float(v_max)
        self.a_i_max = float(a_i_max)
        self.w_max_deg = float(w_max_deg)
        self.yaw_k = float(yaw_k)
        self.tilt_max_deg = float(tilt_max_deg)
        self.rates = (float(centre_rate_deg), float(max_rate_deg), float(rate_expo))
        self.reset()

    def reset(self):
        """Zero the integrator. Called on every arm -- an integrator that wound up while
        the drone sat on its stand is a lurch the moment thrust is applied."""
        self.integral = np.zeros(3)
        self.last = {}
        # measured-force path: filtered applied thrust (specific, m/s^2) and filtered
        # measured acceleration; None until the first tick with an IMU sample.
        self._T_filt = None
        self._xdd_filt = None
        self._u_applied = None

    # ── the loop ─────────────────────────────────────────────────────────────

    def step(self, p, v, q, p_ref, v_ref, a_ff, dt, thrust_ratio,
             heading=0.0, integrate=True, f_imu=None, a_cable=None):
        """One control tick. Returns (roll, pitch, throttle, yaw) with sticks in
        [-1, 1] and throttle in [0, 1].

        `integrate=False` freezes the integrator without zeroing it -- for disarmed, on
        the ground, or landing, where the position error is not the loop's to correct
        and winding up on it produces a kick at the next takeoff.

        `f_imu` is the body-frame specific force from the IMU (thrust + cable, no
        gravity) and `a_cable` the modelled world-frame cable acceleration the reference
        was built with (zeros if absent). Both are only used when indi_gain > 0; with
        indi_gain == 0 the output is exactly the classic loop's.
        """
        p = np.asarray(p, float)
        v = np.asarray(v, float)
        p_ref = np.asarray(p_ref, float)
        v_ref = np.asarray(v_ref, float)
        a_ff = np.asarray(a_ff, float)

        # Position-servo'd velocity command: this is the paper's v_d. v_ref alone would
        # integrate its own error and drift off the formation.
        v_sp = v_ref + self.kp_pos * (p_ref - p)
        n = float(np.linalg.norm(v_sp))
        if n > self.v_max:
            v_sp = v_sp * (self.v_max / n)

        e_v = v_sp - v
        if integrate and self.ki > 0.0:
            self.integral = self.integral + e_v * float(dt)
            # Clamp the CONTRIBUTION, not the raw integral, so the bound stays 2 m/s^2
            # of authority whatever ki is set to.
            lim = self.a_i_max / self.ki
            nrm = float(np.linalg.norm(self.integral))
            if nrm > lim:
                self.integral = self.integral * (lim / nrm)

        a_sp = limit_tilt(a_ff + self.kv * e_v + self.ki * self.integral,
                          self.tilt_max_deg)
        throttle, q_sp = tilt_quat_from_accel(a_sp, thrust_ratio, heading)
        indi = {}
        if self.indi_gain > 0.0 and f_imu is not None:
            throttle, indi = self._indi_throttle(throttle, a_sp, q, f_imu, a_cable,
                                                 float(dt), float(thrust_ratio))
        self._u_applied = throttle

        # Attitude -> body rates. Roll/pitch come from the tilt error; yaw is driven
        # separately toward the held heading so a yaw disagreement never steals
        # authority from the thrust axis.
        e_att = attitude_error(q_sp, q)
        w_cmd = np.degrees(self.k_att * e_att)
        w_cmd = np.clip(w_cmd, -self.w_max_deg, self.w_max_deg)

        c, m, e = self.rates
        roll = betaflight_rates_inv(w_cmd[0], c, m, e)
        pitch = betaflight_rates_inv(w_cmd[1], c, m, e)
        # dynamics.py applies betaflight_rates(-u[3]) for yaw, so the stick is negated.
        yaw = -betaflight_rates_inv(w_cmd[2], c, m, e)

        self.last = {'v_sp': v_sp, 'e_v': e_v, 'a_sp': a_sp, 'q_sp': q_sp,
                     'e_att': e_att, 'w_cmd_deg': w_cmd,
                     'integral_accel': self.ki * self.integral, **indi}
        return float(roll), float(pitch), float(throttle), float(yaw)

    def _indi_throttle(self, throttle_nom, a_sp, q, f_imu, a_cable, dt, thrust_ratio):
        """Incremental thrust magnitude from the measured acceleration (module doc)."""
        R = _quat_to_rot(np.asarray(q, float))
        b = R[:, 2]                                       # body thrust axis, world
        g = np.array([0.0, 0.0, 9.81])
        xdd_des = np.asarray(a_sp, float) - g + (np.zeros(3) if a_cable is None
                                                 else np.asarray(a_cable, float))
        xdd_meas = R @ np.asarray(f_imu, float) - g
        alpha = dt / (self.indi_tau + dt) if self.indi_tau > 0.0 else 1.0
        T_applied = thrust_ratio * (self._u_applied if self._u_applied is not None
                                    else throttle_nom)
        if self._T_filt is None:
            self._T_filt, self._xdd_filt = T_applied, xdd_meas.copy()
        else:
            self._T_filt += alpha * (T_applied - self._T_filt)
            self._xdd_filt += alpha * (xdd_meas - self._xdd_filt)
        inc = float(np.clip(np.dot(xdd_des - self._xdd_filt, b),
                            -self.indi_a_max, self.indi_a_max))
        u_indi = self._T_filt / thrust_ratio + inc / (thrust_ratio * self.indi_slope_ratio)
        u = throttle_nom + self.indi_gain * (u_indi - throttle_nom)
        u = float(np.clip(u, self.indi_thr_min, 0.6))
        return u, {'indi_inc': inc, 'indi_T_filt': float(self._T_filt),
                   'indi_u_nom': float(throttle_nom)}
