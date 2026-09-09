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
