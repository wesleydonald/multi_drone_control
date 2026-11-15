"""The C++ exit's curves and transaction, without ROS (docs/rig/cpp_exit_impl_contract_2026-11-15.md sections 1 and
3-4): the backend's knot and boundary equations ported exactly, the moving prefix (start p/v/a exact at absolute ROS
times, the planner's limits, a lengthened blend on a failed check), the refresh (the protected window copied
unchanged, the junction C2), the relative blends of every cancel and takeover, and the transaction's messages."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from tejen_mission import c1f2_handoff as h
from tejen_mission.reference_generators import TrajectoryReference

T0 = 1_763_200_000.123456           # an absolute ROS time: double spacing ~2.4e-7 s here
V_MAX, A_MAX, J_MAX = np.ones(3), np.array([1.0, 1.0, 1.5]), np.full(3, 4.0)
LIMITS = dict(max_velocity=V_MAX, max_acceleration=A_MAX, max_jerk=J_MAX)
OFFSET = np.array([0.0, 0.2, 1.9])


def circle(radius=0.75, speed=0.25, z=1.0):
    w = speed / radius

    def ring(times):
        t = np.asarray(times, dtype=float) - T0
        c, s = np.cos(w * t), np.sin(w * t)
        zero = 0.0 * t
        return (np.stack([radius * c, radius * s, z + zero], 1), np.stack([-speed * s, speed * c, zero], 1),
                np.stack([-speed * w * c, -speed * w * s, zero], 1))
    return ring


def plus_offset(ring, offset=OFFSET):
    return lambda times: (ring(times)[0] + offset, ring(times)[1], ring(times)[2])


def mismatched_start(target, dp=(0.03, -0.02, 0.01), dv=(0.08, 0.05, 0.0)):
    p, v, _ = (x[0] for x in target(np.array([T0])))
    return p + dp, v + dv, np.zeros(3)


def prefix(target=None, start=None, **kw):
    target = target or plus_offset(circle())
    check = h.fit_moving_prefix(start_time=T0, end_time=T0 + 5.0, start_state=start or mismatched_start(target),
                                target=target, blend_s=kw.pop('blend_s', 1.0), **{**LIMITS, **kw})
    return check, target


def states_close(a, b, tol):
    return max(float(np.max(np.abs(x - y))) for x, y in zip(a, b)) <= tol


# ---- the backend's equations -----------------------------------------------------------------------------------------

def test_open_uniform_knots_are_the_backends():
    np.testing.assert_array_equal(h.open_uniform_knots(0.0, 1.0, 4),
                                  [0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1])
    with pytest.raises(ValueError):
        h.open_uniform_knots(1.0, 1.0, 4)


def test_the_first_three_control_points_give_the_start_state_exactly():
    rng = np.random.default_rng(3)
    knots = h.open_uniform_knots(T0, T0 + 2.0, 8)
    p, v, a = rng.normal(size=3) + [0, 1, 2.9], rng.normal(size=3) * 0.3, rng.normal(size=3) * 0.5
    q = np.vstack([h.initial_control_points(p, v, a, knots), rng.normal(size=(8, 3))])
    got = h.SplinePiece(knots, q).state([T0])
    assert states_close([x[0] for x in got[:3]], (p, v, a), 1e-12)


# ---- the prefix ------------------------------------------------------------------------------------------------------

def test_prefix_starts_on_the_executed_state_and_follows_the_ring_after_the_blend():
    check, target = prefix()
    assert check.ok and check.attempts == 1 and check.start_error <= h.EXIT_CONTINUITY_TOLERANCE
    spline = h.CommittedSpline([check.piece])
    assert (spline.start, spline.end) == (T0, T0 + 5.0)
    assert states_close([x[0] for x in spline.state([T0])[:3]], mismatched_start(target), 1e-9)
    after = np.linspace(T0 + 1.2, T0 + 5.0, 40)
    assert np.max(np.abs(spline.state(after)[0] - target(after)[0])) < 2e-3       # the fit, reported not tested
    times = np.linspace(T0, T0 + 5.0, 2001)
    _, v, a, j = spline.state(times)
    assert np.all(np.abs(v) <= V_MAX + 1e-9) and np.all(np.abs(a) <= A_MAX + 1e-9) and np.all(np.abs(j) <= J_MAX + 1e-9)


def test_a_failed_check_lengthens_the_blend_and_an_impossible_one_gives_no_prefix():
    target = plus_offset(circle())
    sharp = mismatched_start(target, dp=(0.0, 0.0, 0.0), dv=(0.15, 0.0, 0.0))
    check, _ = prefix(target, sharp, max_jerk=np.full(3, 2.0))
    assert check.ok and check.attempts > 1 and check.blend_s > 1.0
    fast = plus_offset(circle(speed=1.2))            # the ring itself beyond his 1 m/s
    check, _ = prefix(fast, mismatched_start(fast))
    assert not check.ok and check.reason == 'VELOCITY_LIMIT' and check.attempts == h.EXIT_FIT_ATTEMPTS


def test_refresh_copies_the_protected_window_unchanged_and_joins_c2():
    check, target = prefix()
    bound = h.CommittedSpline([check.piece])
    t_r = T0 + 1.0
    new, fit = h.refresh_moving_prefix(bound=bound, refresh_time=t_r, protected_s=1.5, end_time=t_r + 5.0,
                                       target=target, blend_s=1.0, **LIMITS)
    assert new is not None and fit.ok and (new.start, new.end) == (t_r, t_r + 5.0)
    copied = new.pieces[0]
    np.testing.assert_array_equal(copied.knots, check.piece.knots)          # the same knots and coefficients
    np.testing.assert_array_equal(copied.control_points, check.piece.control_points)
    # the backend's identity test, [t_R, t_R + Pw] at 21 times to 1e-9: the copies run past its end, so exactly equal
    window = np.linspace(t_r, t_r + 1.5, 21)
    assert states_close(bound.state(window)[:3], new.state(window)[:3], 0.0)
    junction = t_r + 1.5 + h.EXIT_JUNCTION_MARGIN_S
    assert new.pieces[-1].valid_from == junction > t_r + 1.5
    assert states_close(new.pieces[-1].state([junction])[:3], bound.state([junction])[:3], h.EXIT_CONTINUITY_TOLERANCE)
    again, _ = h.refresh_moving_prefix(bound=new, refresh_time=t_r + 1.0, protected_s=1.5, end_time=t_r + 6.0,
                                       target=target, blend_s=1.0, **LIMITS)
    window = np.linspace(t_r + 1.0, t_r + 2.5, 21)
    assert again is not None and states_close(new.state(window)[:3], again.state(window)[:3], 0.0)


def test_refresh_needs_the_protected_window_inside_the_bound_prefix():
    check, target = prefix()
    bound = h.CommittedSpline([check.piece])
    new, reason = h.refresh_moving_prefix(bound=bound, refresh_time=T0 + 4.0, protected_s=1.5, end_time=T0 + 9.0,
                                          target=target, blend_s=1.0, **LIMITS)
    assert new is None and reason == 'PROTECTED_WINDOW_NOT_COVERED'


def test_message_round_trip_is_the_backends_suffix_replacement():
    check, target = prefix()
    bound = h.CommittedSpline([check.piece])
    new, _ = h.refresh_moving_prefix(bound=bound, refresh_time=T0 + 1.0, protected_s=1.5, end_time=T0 + 6.0,
                                     target=target, blend_s=1.0, **LIMITS)
    msg = NS(pieces=[NS(valid_from_s=p.valid_from, valid_until_s=p.valid_until, knots=list(p.knots),
                        control_points=[NS(x=q[0], y=q[1], z=q[2]) for q in p.control_points]) for p in new.pieces])
    back = h.CommittedSpline.from_message(msg)
    times = np.linspace(T0 + 1.0, T0 + 6.0, 301)
    assert states_close(back.state(times), new.state(times), 0.0)
    broken = h.SplinePiece(new.pieces[-1].knots, new.pieces[-1].control_points + [0.0, 0.0, 0.01])
    with pytest.raises(ValueError, match='C2'):
        h.CommittedSpline([new.pieces[0], broken])


# ---- the relative blends ---------------------------------------------------------------------------------------------

def test_relative_blend_starts_on_the_error_and_ends_at_exactly_zero():
    error = (np.array([0.02, -0.01, 0.0]), np.array([0.1, 0.0, 0.0]), np.array([0.2, 0.0, 0.0]))
    still = lambda tau: np.zeros((len(tau), 3))          # noqa: E731
    check = h.fit_relative_blend(error_state=error, blend_s=1.0, target_velocity=still, target_acceleration=still,
                                 **LIMITS)
    assert check.ok and check.start_error <= h.EXIT_CONTINUITY_TOLERANCE
    end = check.piece.state([check.piece.valid_until, 10.0])
    assert all(np.all(x == 0.0) for x in end[:3])
    moving = lambda tau: np.tile([0.95, 0.0, 0.0], (len(tau), 1))   # noqa: E731
    check = h.fit_relative_blend(error_state=error, blend_s=1.0, target_velocity=moving, target_acceleration=still,
                                 **LIMITS)
    assert not check.ok and check.reason == 'VELOCITY_LIMIT'


def test_relative_grant_state_needs_no_stop():
    ready = h.relative_grant_state(reference_position=[0, 0, 2.9], reference_velocity=[0.25, 0, 0],
                                   reference_acceleration=[0, 0.08, 0], measured_position=[0.02, 0, 2.9],
                                   measured_velocity=[0.3, 0, 0], measured_acceleration=[0, 0.1, 0],
                                   max_drift_m=0.3, settle_speed_mps=0.12, settle_acceleration_mps2=0.5)
    assert ready.ready
    lagging = h.relative_grant_state(reference_position=[0, 0, 2.9], reference_velocity=[0.25, 0, 0],
                                     reference_acceleration=[0, 0, 0], measured_position=[0, 0, 2.9],
                                     measured_velocity=[0.0, 0, 0], measured_acceleration=[0, 0, 0],
                                     max_drift_m=0.3, settle_speed_mps=0.12, settle_acceleration_mps2=0.5)
    assert not lagging.ready


# ---- the transaction -------------------------------------------------------------------------------------------------

def bound_transaction():
    check, _ = prefix()
    tx = h.MovingExitHandoff()
    tx.start(now_s=T0 - 0.5, now_ns=round((T0 - 0.5) * 1e9), timeout_s=7.0)
    tx.open(request_id='a' * 32, now_s=T0, now_ns=round(T0 * 1e9))
    tx.bind(h.CommittedSpline([check.piece]), stamp_ns=round(T0 * 1e9), now_s=T0)
    return tx


def test_requests_reset_once_then_each_sequence_once_with_the_fixed_deadline():
    tx = bound_transaction()
    deadline = tx.deadline_ns
    assert deadline == round((T0 - 0.5) * 1e9) + 7_000_000_000
    assert tx.requests() == ['a' * 32 + ' reset', f'{"a" * 32} seedtraj exit 1 {round(T0 * 1e9)} {deadline}']
    assert tx.requests() == []
    tx.bind(tx.prefix, stamp_ns=round((T0 + 1) * 1e9), now_s=T0 + 1)
    assert tx.requests() == [f'{"a" * 32} seedtraj exit 2 {round((T0 + 1) * 1e9)} {deadline}']
    assert tx.deadline_ns == deadline
    assert not tx.grant_ready()
    tx.observe_status({'request_id': 'a' * 32, 'epoch': 3, 'stamp_ns': 1, 'prepared': True, 'granted': False,
                       'reference_stamp_ns': 0, 'exit_reject_reason': ''}, now_s=T0 + 1)
    assert tx.grant_ready() and tx.reject_reason() == ''
    tx.observe_status({'request_id': 'b' * 32, 'epoch': 4, 'stamp_ns': 2, 'prepared': False, 'granted': False,
                       'reference_stamp_ns': 0, 'exit_reject_reason': 'prefix_missing'}, now_s=T0 + 1)
    assert tx.reject_reason() == ''                            # another transaction's status is not ours
    assert tx.close('failed', 'timeout') == 'a' * 32 + ' cancel' and tx.close('failed') is None


def test_a_window_joins_only_if_it_is_the_prefix_at_the_same_times_up_to_the_join():
    tx = bound_transaction()
    stamp = round((T0 + 0.4) * 1e9)
    times = stamp * 1e-9 + np.arange(61) / 30.0
    p, v, a, _ = tx.prefix.state(times)
    join = stamp * 1e-9 + 0.3
    after = times > join
    off = p.copy()
    off[after] += [0.0, 0.05, 0.0]                             # the C++ plan after its splice
    tx.status = {'splice_time_ns': round(join * 1e9)}
    ok, errors = tx.window_matches(TrajectoryReference(off, v, a), stamp_ns=stamp, dt=1 / 30)
    assert ok and max(errors) < 1e-9
    late, _, _, _ = tx.prefix.state(times + 0.03)              # a clock-shifted copy of the same prefix
    ok, errors = tx.window_matches(TrajectoryReference(late, v, a), stamp_ns=stamp, dt=1 / 30)
    assert not ok and errors[0] > 1e-3
    tx.status = {}
    ok, _ = tx.window_matches(TrajectoryReference(off, v, a), stamp_ns=stamp, dt=1 / 30)
    assert ok                                                  # no join reported: sample 0 alone
    tx.status = {'splice_time_ns': stamp - 1_000_000}
    ok, _ = tx.window_matches(TrajectoryReference(off, v, a), stamp_ns=stamp, dt=1 / 30)
    assert not ok                                              # stamped after its join: no prefix in it


def test_a_window_planned_on_the_previous_prefix_still_joins_after_a_refresh():
    tx = bound_transaction()
    old = tx.prefix
    target = plus_offset(circle())
    t_r = T0 + 1.0
    new, _ = h.refresh_moving_prefix(bound=old, refresh_time=t_r, protected_s=1.5, end_time=t_r + 5.0, target=target,
                                     blend_s=1.0, **LIMITS)
    stamp = round((t_r - 0.01) * 1e9)                         # published just before the refresh
    times = stamp * 1e-9 + np.arange(61) / 30.0
    p, v, a, _ = old.state(times)
    tx.bind(new, stamp_ns=round(t_r * 1e9), now_s=t_r)
    tx.status = {'splice_time_ns': stamp + 300_000_000}
    ok, errors = tx.window_matches(TrajectoryReference(p, v, a), stamp_ns=stamp, dt=1 / 30)
    assert ok and max(errors) == 0.0
