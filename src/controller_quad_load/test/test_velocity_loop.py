"""
Unit tests for velocity_loop.py — THESIS_PLAN §12.1 Stage V, ladder step V-a.

Design note: docs/design/velocity_loop.md.

Two of these are worth more than the rest, because they are the ones that pin this
module against code it REIMPLEMENTS rather than imports:

  * the Betaflight rate curve, against the CasADi expression in dynamics.py
  * the accel -> (throttle, quaternion) map, against acados.py's version

Both were reimplemented deliberately -- velocity_loop.py has to stay importable without
a compiled acados solver so the SIL bench and these tests can drive it -- and a
reimplementation that drifts from its original is exactly the failure this project has
been bitten by before (`thrust_quad_c` stale at 203, `CABLE_LEN` at 0.6).
"""
import math
import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from controller_quad_load.velocity_loop import (          # noqa: E402
    VelocityLoop, attitude_error, betaflight_rates, betaflight_rates_inv,
    limit_tilt, tilt_quat_from_accel)

G = 9.81
KT = 32.9
LEVEL = np.array([1.0, 0.0, 0.0, 0.0])


# ── the reimplementations, pinned against their originals ────────────────────

def test_rate_curve_matches_the_casadi_model_in_dynamics_py():
    """The forward curve must be the SAME curve the MPC's model uses. If these drift,
    the velocity loop and the MPC are flying different aircraft and the comparison
    between them means nothing.

    The tolerance is 1e-3 deg/s, not machine precision, and the difference is not a
    disagreement. dynamics.py smooths the absolute value as sqrt(x*x + 1e-6) because
    the solver needs a differentiable |x|; this module and the actual Betaflight
    firmware use the exact one. Measured over the full stick range the gap peaks at
    9.0e-4 deg/s (at full stick, 1.5e-4 relative) -- six orders below the 670 deg/s the
    curve spans, and the centre-rate term is exact everywhere because sgn*ax == x."""
    import casadi as cs
    from controller_quad_load.dynamics import QuadLoadDynamics

    d = QuadLoadDynamics()
    x = cs.MX.sym('x')
    f = cs.Function('f', [x, d.centre_rate_deg, d.max_rate_deg, d.rate_expo],
                    [d.betaflight_rates(x)])
    for stick in np.linspace(-1.0, 1.0, 401):
        assert float(f(stick, 70.0, 670.0, 0.5)) == pytest.approx(
            betaflight_rates(stick), abs=1e-3)


def test_tilt_quat_matches_the_acados_version():
    """velocity_loop and the MPC must agree on what a required acceleration means as
    (throttle, attitude), or a mode switch is a step change in command."""
    from controller_quad_load.acados import _tilt_quat_from_accel

    for a in ([0.0, 0.0, G], [1.5, 0.0, G], [0.0, -2.0, G], [3.0, 2.0, 11.0]):
        for heading in (0.0, 1.2, -2.5):
            t_ref, q_ref = _tilt_quat_from_accel(np.array(a), KT, heading)
            t_new, q_new = tilt_quat_from_accel(a, KT, heading)
            assert t_new == pytest.approx(t_ref, abs=1e-12)
            assert np.allclose(q_new, q_ref, atol=1e-12)


def test_rates_inverse_round_trips_to_machine_precision():
    """A rate-loop gain error is indistinguishable in flight from a bad attitude gain,
    so we would tune the wrong thing. Exact, not approximately right."""
    for stick in np.linspace(-1.0, 1.0, 201):
        assert betaflight_rates_inv(betaflight_rates(stick)) == pytest.approx(
            stick, abs=1e-9)


def test_rates_inverse_saturates_rather_than_extrapolating():
    """Beyond the curve's range the answer is full stick, not an extrapolated one."""
    assert betaflight_rates_inv(5000.0) == 1.0
    assert betaflight_rates_inv(-5000.0) == -1.0


# ── attitude error ───────────────────────────────────────────────────────────

def test_attitude_error_of_a_known_roll_is_that_angle():
    ang = math.radians(10.0)
    q = np.array([math.cos(ang / 2), math.sin(ang / 2), 0.0, 0.0])
    e = attitude_error(q, LEVEL)
    assert e[0] == pytest.approx(ang, abs=1e-9)
    assert abs(e[1]) < 1e-12 and abs(e[2]) < 1e-12


def test_attitude_error_takes_the_SHORT_way_round():
    """A quaternion and its negation are the same attitude. Without the sign fix, an
    attitude 1 degree away on the far side of the double cover commands a 359 degree
    slew -- on a drone carrying a load, a flip."""
    ang = math.radians(1.0)
    q = np.array([math.cos(ang / 2), math.sin(ang / 2), 0.0, 0.0])
    short = attitude_error(q, LEVEL)
    assert np.allclose(attitude_error(-q, LEVEL), short, atol=1e-9)
    assert np.linalg.norm(short) < math.radians(2.0)


def test_attitude_error_is_zero_at_the_setpoint():
    assert np.allclose(attitude_error(LEVEL, LEVEL), np.zeros(3), atol=1e-12)


# ── the loop ─────────────────────────────────────────────────────────────────

def hover_args(**kw):
    a = dict(p=[0, 0, 1.0], v=[0, 0, 0], q=LEVEL, p_ref=[0, 0, 1.0],
             v_ref=[0, 0, 0], a_ff=[0, 0, G], dt=0.02, thrust_ratio=KT)
    a.update(kw)
    return a


def test_hover_at_the_reference_is_a_fixed_point():
    """On its reference, level, at rest: hover throttle and no stick. Anything else is
    a bias that would fly the drone off its setpoint the moment the mode is enabled."""
    roll, pitch, thr, yaw = VelocityLoop().step(**hover_args())
    assert thr == pytest.approx(G / KT, abs=1e-9)
    assert (abs(roll), abs(pitch), abs(yaw)) == pytest.approx((0, 0, 0), abs=1e-9)


def test_a_position_error_tilts_thrust_TOWARDS_the_reference():
    """The sign check. A drone 0.3 m short of its reference in +x must tilt to
    accelerate in +x; the opposite sign is a runaway, not a slow response."""
    loop = VelocityLoop()
    loop.step(**hover_args(p=[-0.3, 0.0, 1.0]))
    a_sp = loop.last['a_sp']
    assert a_sp[0] > 0.0
    assert loop.last['v_sp'][0] > 0.0


def test_a_velocity_reference_is_followed_without_a_position_error():
    """Tracking a moving reference is the whole point: sitting exactly on a reference
    that is moving at 0.6 m/s must still command the acceleration to keep up."""
    loop = VelocityLoop()
    loop.step(**hover_args(v_ref=[0.6, 0.0, 0.0], v=[0.0, 0.0, 0.0]))
    assert loop.last['e_v'][0] == pytest.approx(0.6, abs=1e-9)
    assert loop.last['a_sp'][0] > 0.0


def test_the_velocity_command_is_clamped():
    loop = VelocityLoop(v_max=1.5)
    loop.step(**hover_args(p=[-50.0, 0.0, 1.0]))
    assert np.linalg.norm(loop.last['v_sp']) == pytest.approx(1.5, abs=1e-9)


def test_the_integrator_is_bounded_by_its_configured_authority():
    """Anti-windup. A sustained error must not be able to command more than a_i_max of
    integral acceleration, however long it persists."""
    loop = VelocityLoop(a_i_max=2.0, ki=1.0)
    for _ in range(5000):
        loop.step(**hover_args(p=[-5.0, 0.0, 1.0]))
    assert np.linalg.norm(loop.last['integral_accel']) <= 2.0 + 1e-9


def test_the_integrator_freezes_when_asked_and_does_not_forget():
    """Frozen, not zeroed: on the ground and during landing the position error is not
    the loop's to correct, but discarding the trim it had learned in flight would step
    the command."""
    loop = VelocityLoop()
    for _ in range(50):
        loop.step(**hover_args(p=[-0.5, 0.0, 1.0]))
    held = loop.integral.copy()
    assert np.linalg.norm(held) > 0
    for _ in range(50):
        loop.step(**hover_args(p=[-0.5, 0.0, 1.0], integrate=False))
    assert np.allclose(loop.integral, held, atol=1e-12)


def test_reset_clears_the_integrator():
    """Called on every arm. An integrator that wound up on the stand is a lurch the
    moment thrust is applied."""
    loop = VelocityLoop()
    for _ in range(50):
        loop.step(**hover_args(p=[-0.5, 0.0, 1.0]))
    loop.reset()
    assert np.allclose(loop.integral, np.zeros(3), atol=1e-12)


def test_zero_gains_reduce_to_the_planner_feedforward():
    """With all feedback off the loop must emit exactly the feedforward the planner
    asked for -- the property that makes it comparable against the MPC path."""
    loop = VelocityLoop(kp_pos=0.0, kv=0.0, ki=0.0)
    _, _, thr, _ = loop.step(**hover_args(p=[-1.0, 0.0, 1.0], a_ff=[2.0, 0.0, G]))
    assert np.allclose(loop.last['a_sp'], [2.0, 0.0, G], atol=1e-12)
    assert thr == pytest.approx(math.hypot(2.0, G) / KT, abs=1e-9)


def test_yaw_stick_sign_matches_the_model_convention():
    """dynamics.py applies betaflight_rates(-u[3]) for yaw, so a positive commanded yaw
    rate needs a NEGATIVE stick. Getting this backwards yaws the wrong way, which on a
    fleet with yaw-aware slot assignment scrambles the formation."""
    ang = math.radians(10.0)
    q_meas = np.array([math.cos(ang / 2), 0.0, 0.0, -math.sin(ang / 2)])
    loop = VelocityLoop()
    _, _, _, yaw = loop.step(**hover_args(q=q_meas))
    assert loop.last['w_cmd_deg'][2] > 0.0
    assert yaw < 0.0


def test_the_stick_this_produces_lands_the_rate_the_PLANT_actually_flies():
    """End-to-end, across the one place these two curves differ.

    `betaflight_rates_inv` inverts the EXACT curve, which is what real Betaflight runs.
    The SIL plant, the Gazebo betaflight emulator and the MPC's model all use the
    solver-smoothed `sqrt(x*x + 1e-6)` variant. So the rate the plant actually flies is
    not identically the rate that was asked for, and the question is whether the gap
    matters. It does not: bounded by 1e-3 deg/s against a 670 deg/s range."""
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(HERE))), 'tools'))
    from sil.plant import betaflight_rates as plant_rates

    for asked in np.linspace(-660.0, 660.0, 265):
        stick = betaflight_rates_inv(asked)
        assert plant_rates(stick) == pytest.approx(asked, abs=1e-3)


# ── tilt limiting ────────────────────────────────────────────────────────────

def test_tilt_limit_caps_the_commanded_lean_and_keeps_the_vertical_part():
    """The failure R0067 hit: throttle clamps but the horizontal term does not, so a
    drone that cannot reach its reference leans further and further. Scale the
    HORIZONTAL part — losing altitude to chase a lateral error is how a carrying fleet
    drops its load."""
    a = limit_tilt([50.0, 0.0, G], 30.0)
    assert a[2] == pytest.approx(G, abs=1e-12)
    assert math.degrees(math.atan2(a[0], a[2])) == pytest.approx(30.0, abs=1e-9)


def test_tilt_limit_leaves_a_modest_command_untouched():
    a = limit_tilt([1.0, 0.0, G], 30.0)
    assert np.allclose(a, [1.0, 0.0, G], atol=1e-12)


def test_tilt_limit_never_commands_inverted_thrust():
    """A large downward correction must not flip the thrust axis."""
    a = limit_tilt([2.0, 0.0, -30.0], 30.0)
    assert a[2] > 0.0


def test_the_loop_respects_its_tilt_limit_under_a_huge_error():
    loop = VelocityLoop(tilt_max_deg=25.0)
    for _ in range(200):
        loop.step(**hover_args(p=[-50.0, 0.0, 1.0]))
    a = loop.last['a_sp']
    assert math.degrees(math.atan2(math.hypot(a[0], a[1]), a[2])) <= 25.0 + 1e-6


# ── measured-force (INDI) throttle ───────────────────────────────────────────

def test_indi_off_is_byte_identical_to_the_classic_loop():
    """indi_gain 0 must not touch the output even when an IMU sample is supplied: every
    verified velocity-mode result was flown without it."""
    a = hover_args(p=[0.1, -0.2, 0.9], v=[0.1, 0, 0])
    ref = VelocityLoop().step(**a)
    got = VelocityLoop(indi_gain=0.0).step(**a, f_imu=[0, 0, 12.0], a_cable=[0, 0, -2.0])
    assert got == ref


def _hover_sim(kt_assumed, kt_true, indi_gain, seconds=8.0, dt=0.02, a_cable_true=-3.0):
    """Point-mass vertical hover on a cable with a WRONG thrust gain: the loop believes
    kt_assumed, the plant flies kt_true. Attitude is taken as perfect (level), so only
    the throttle path is under test. Returns the final height error (m)."""
    loop = VelocityLoop(ki=0.0, indi_gain=indi_gain, indi_tau=0.05)
    z, vz = 1.0, 0.0
    a_cable_model = -3.0                      # the reference's cable model (exact here)
    a_ff = [0.0, 0.0, G - a_cable_model]      # a_ff = g + 0 - a_cable
    f_imu = [0.0, 0.0, G]                     # at rest: thrust + cable = g
    for _ in range(int(seconds / dt)):
        _, _, thr, _ = loop.step([0, 0, z], [0, 0, vz], LEVEL, [0, 0, 1.0], [0, 0, 0],
                                 a_ff, dt, kt_assumed, f_imu=f_imu,
                                 a_cable=[0, 0, a_cable_model])
        a_thrust = kt_true * thr
        zdd = a_thrust + a_cable_true - G
        vz += zdd * dt
        z += vz * dt
        f_imu = [0.0, 0.0, a_thrust + a_cable_true]
    return z - 1.0


def test_a_wrong_thrust_gain_is_a_steady_offset_without_indi():
    """The classic loop has no integrator here (vel_ki 0, the §10 conclusion): a 15 %
    over-estimated kT parks the drone below its reference for good."""
    err = _hover_sim(kt_assumed=1.15 * KT, kt_true=KT, indi_gain=0.0)
    assert err < -0.05


def test_indi_removes_the_steady_offset_from_a_wrong_thrust_gain():
    err = _hover_sim(kt_assumed=1.15 * KT, kt_true=KT, indi_gain=1.0)
    assert abs(err) < 0.01


def test_indi_removes_the_offset_from_a_wrong_cable_model_too():
    """The reference believes the cable pulls 3 m/s^2, the plant's cable pulls 4 (a
    25 % heavier load than the controller was told -- the fatal mis-seed direction)."""
    loop_err = _hover_sim(kt_assumed=KT, kt_true=KT, indi_gain=1.0, a_cable_true=-4.0)
    assert abs(loop_err) < 0.01
    classic = _hover_sim(kt_assumed=KT, kt_true=KT, indi_gain=0.0, a_cable_true=-4.0)
    assert classic < -0.05


def test_indi_increment_is_bounded():
    loop = VelocityLoop(indi_gain=1.0, indi_a_max=1.0)
    a = hover_args()
    loop.step(**a, f_imu=[0, 0, 30.0])       # absurd IMU sample
    assert abs(loop.last['indi_inc']) <= 1.0 + 1e-9


def _hover_sim_quadratic(indi_slope_ratio, seconds=8.0, dt=0.02):
    """Same hover on a QUADRATIC plant a = c*u^2 (the sim's motor model), with the loop's
    kT the secant at hover and the increment's control effectiveness set by
    indi_slope_ratio. Returns (final height error, throttle std over the last 2 s)."""
    a_cable = -3.0
    a_hover = G - a_cable                      # specific thrust at hover
    u_hover = 0.4
    c = a_hover / u_hover ** 2
    kt_secant = c * u_hover
    loop = VelocityLoop(ki=0.0, indi_gain=1.0, indi_tau=0.05, indi_slope_ratio=indi_slope_ratio)
    z, vz = 0.9, 0.0                            # start 10 cm low
    f_imu = [0.0, 0.0, G]
    thr_hist = []
    for k in range(int(seconds / dt)):
        _, _, thr, _ = loop.step([0, 0, z], [0, 0, vz], LEVEL, [0, 0, 1.0], [0, 0, 0],
                                 [0, 0, a_hover], dt, kt_secant, f_imu=f_imu,
                                 a_cable=[0, 0, a_cable])
        a_thrust = c * thr ** 2
        vz += (a_thrust + a_cable - G) * dt
        z += vz * dt
        f_imu = [0.0, 0.0, a_thrust + a_cable]
        thr_hist.append(thr)
    tail = np.array(thr_hist[-int(2.0 / dt):])
    return z - 1.0, float(tail.std())


def test_slope_ratio_two_is_calm_on_the_quadratic_plant():
    """With the increment scaled by the tangent (2x the secant for a = c*u^2) the hover
    settles without throttle chatter; with the secant (ratio 1) the increment loop runs
    at twice its design gain and the throttle is visibly noisier (SIL R0241)."""
    err2, std2 = _hover_sim_quadratic(2.0)
    err1, std1 = _hover_sim_quadratic(1.0)
    assert abs(err2) < 0.01 and std2 < 0.002
    assert std1 > std2


def test_indi_never_cuts_thrust_below_its_floor():
    """A rod that pushes the drone up reads as a large upward measured acceleration; the
    increment wants zero thrust, the floor keeps rate authority (Gazebo R0242)."""
    loop = VelocityLoop(indi_gain=1.0, indi_thr_min=0.2)
    thr = None
    for _ in range(20):
        _, _, thr, _ = loop.step(**hover_args(), f_imu=[0, 0, 40.0])
    assert thr == 0.2


# ── anti-swing ───────────────────────────────────────────────────────────────

def _pendulum_sim(indi_gain, swing_k, seconds=12.0, dt=0.02, l=0.5):
    """Planar pendulum (load) under a drone the loop positions in x. The drone's thrust
    is taken as exact (the INDI claim), so its acceleration IS the loop's a_sp; the rod
    force on the drone is what INDI rejects. Returns (initial, final) swing amplitude."""
    loop = VelocityLoop(ki=0.0, indi_gain=indi_gain, swing_k=swing_k)
    px, vx = 0.0, 0.0                    # drone
    th, dth = 0.25, 0.0                  # load angle from vertical, rad
    amp0, amp = 0.25, 0.0
    peaks = []
    for k in range(int(seconds / dt)):
        # load position/velocity from the pendulum geometry
        lx, lvx = px + l * math.sin(th), vx + l * dth * math.cos(th)
        r, pch, thr, y = loop.step([px, 0, 1.0], [vx, 0, 0], LEVEL, [0, 0, 1.0], [0, 0, 0],
                                   [0, 0, G], dt, KT, v_load=[lvx, 0.0, 0.0])
        ax = float(loop.last['a_sp'][0])        # exact thrust: drone accel = command
        # pendulum driven by the pivot acceleration
        ddth = -(G / l) * math.sin(th) - (ax / l) * math.cos(th) - 0.02 * dth
        dth += ddth * dt; th += dth * dt
        vx += ax * dt; px += vx * dt
        if k > int(seconds / dt) - int(2.0 / dt):
            peaks.append(abs(th))
    return amp0, max(peaks)


def test_a_stiff_pivot_leaves_the_load_swinging():
    """Exact thrust + no anti-swing = an undamped pendulum: the amplitude survives."""
    a0, a = _pendulum_sim(indi_gain=1.0, swing_k=0.0)
    assert a > 0.6 * a0


def test_anti_swing_damps_the_load():
    a0, a = _pendulum_sim(indi_gain=1.0, swing_k=0.3)
    assert a < 0.15 * a0


def test_anti_swing_has_a_stability_window():
    """Too much of it and the pivot overshoots the bob: the swing GROWS. The gain is a
    window (0.3 with kp 2 / kv 4 here), not a knob to turn up."""
    a0, a_hi = _pendulum_sim(indi_gain=1.0, swing_k=1.0)
    _, a_ok = _pendulum_sim(indi_gain=1.0, swing_k=0.3)
    assert a_hi > a0 and a_ok < 0.15 * a0


def test_swing_term_off_is_byte_identical():
    a = hover_args(p=[0.1, -0.2, 0.9], v=[0.1, 0, 0])
    assert VelocityLoop().step(**a) == VelocityLoop(swing_k=0.0).step(**a, v_load=[1.0, 0, 0])
