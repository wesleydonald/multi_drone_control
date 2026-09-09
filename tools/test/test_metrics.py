"""
Unit tests for tools/metrics.py — THESIS_PLAN §9.2.

Every metric is checked against a SYNTHETIC signal whose answer is known analytically,
not against recorded data. That is the point: recorded data can only tell you a metric
is stable, never that it is correct, and these numbers go straight into the thesis.

Each test names the property it protects in plain language (§8.2).
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import metrics as M  # noqa: E402


# ── tracking accuracy ────────────────────────────────────────────────────────

def test_payload_rmse_of_a_known_constant_offset_is_that_offset():
    """A payload held exactly 0.2 m off its reference has an RMSE of 0.2 m -- RMS of the
    error NORM, not of one axis."""
    n = 100
    desired = np.zeros((n, 3))
    actual = np.tile([0.12, 0.16, 0.0], (n, 1))     # 3-4-5 triangle -> norm 0.2
    assert M.payload_rmse(actual, desired) == pytest.approx(0.2, abs=1e-9)


def test_payload_rmse_is_rms_not_mean():
    """Half the run at 0 error and half at 0.4 gives RMS 0.283, not the mean 0.2.
    Getting this wrong would flatter every result by ~30%."""
    desired = np.zeros((100, 3))
    actual = np.zeros((100, 3))
    actual[50:, 0] = 0.4
    assert M.payload_rmse(actual, desired) == pytest.approx(0.4 / math.sqrt(2), abs=1e-9)


def test_radius_ratio_of_a_shrunken_circle_is_the_shrink_factor():
    """A commanded circle of r=0.5 flown at r=0.374 gives 0.748 -- the documented
    dissipative tracking deficit."""
    th = np.linspace(0, 4 * np.pi, 400, endpoint=False)
    desired = np.column_stack([0.5 * np.cos(th), 0.5 * np.sin(th)])
    actual = np.column_stack([0.374 * np.cos(th), 0.374 * np.sin(th)])
    assert M.radius_ratio(actual, desired) == pytest.approx(0.374 / 0.5, rel=1e-6)


def test_radius_ratio_ignores_a_constant_position_offset():
    """A circle of the right size flown 0.3 m to one side is a POSITION error, not a
    radius error, and must not be reported as one."""
    th = np.linspace(0, 4 * np.pi, 400, endpoint=False)
    desired = np.column_stack([0.5 * np.cos(th), 0.5 * np.sin(th)])
    actual = desired + np.array([0.3, 0.0])
    # measured about the DESIRED centre, a displaced circle has the same mean radius
    # to first order; assert it stays close to 1 rather than reporting a fake deficit
    assert M.radius_ratio(actual, desired) == pytest.approx(1.0, abs=0.2)


def test_phase_lag_recovers_a_known_delay():
    """A sine delayed by exactly 0.6 s reports +0.6 s. Positive means the signal LAGS
    the reference -- the sign the whole tracking investigation is stated in."""
    dt = 0.01
    t = np.arange(0, 20, dt)
    f = 0.2
    ref = np.sin(2 * np.pi * f * t)
    lag = 0.6
    sig = np.sin(2 * np.pi * f * (t - lag))
    assert M.phase_lag_s(t, ref, sig) == pytest.approx(lag, abs=0.02)


def test_phase_lag_reports_zero_for_an_undelayed_signal():
    dt = 0.01
    t = np.arange(0, 20, dt)
    ref = np.sin(2 * np.pi * 0.2 * t)
    assert M.phase_lag_s(t, ref, 0.5 * ref) == pytest.approx(0.0, abs=0.02)


def test_phase_lag_beats_the_sample_period():
    """At 10 Hz logging, rounding the lag to a sample is a 0.05 s error on a 0.589 s
    quantity -- 8%. The parabolic interpolation has to do better than the grid."""
    dt = 0.1
    t = np.arange(0, 40, dt)
    ref = np.sin(2 * np.pi * 0.15 * t)
    sig = np.sin(2 * np.pi * 0.15 * (t - 0.589))
    got = M.phase_lag_s(t, ref, sig)
    assert got == pytest.approx(0.589, abs=0.03)
    assert abs(got - round(got / dt) * dt) > 1e-6, 'answer is stuck on the sample grid'


def test_phase_lag_does_not_alias_by_whole_periods():
    """Every trajectory this metric is used on is periodic, so the cross-correlation has
    EQUAL peaks at lag, lag+T, lag-T... and argmax chooses between them on floating-point
    noise. Unbounded, a 0.6 s delay on a 5 s circle reported 10.637 s (= lag + 2T) and a
    0 s stage reported -21.04 s (= -4T). Eight periods of record, so there is plenty of
    wrong answer available if the search is not capped at half a period."""
    dt = 0.01
    t = np.arange(0, 40, dt)                         # 8 periods at 0.2 Hz
    ref = np.sin(2 * np.pi * 0.2 * t)
    sig = np.sin(2 * np.pi * 0.2 * (t - 0.6))
    assert M.phase_lag_s(t, ref, sig) == pytest.approx(0.6, abs=0.02)


def test_phase_lag_is_stable_against_record_length():
    """The same delay measured over 10 s and 80 s must give the same answer. It did not
    when each lag was normalised by its overlap COUNT: that inflates the variance of the
    edge lags, so how much record you happened to log moved the reported number."""
    dt = 0.02
    got = [M.phase_lag_s(np.arange(0, T, dt),
                         np.sin(2 * np.pi * 0.19 * np.arange(0, T, dt)),
                         np.sin(2 * np.pi * 0.19 * (np.arange(0, T, dt) - 0.42)))
           for T in (10, 20, 40, 80)]
    assert max(got) - min(got) < 0.02, f'lag drifts with record length: {got}'
    assert all(g == pytest.approx(0.42, abs=0.02) for g in got)


def test_dominant_period_of_a_known_circle():
    """phase_lag_s derives its own search bound from this, so a wrong period silently
    re-opens the aliasing above."""
    t = np.arange(0, 60, 0.02)
    assert M.dominant_period_s(t, np.sin(2 * np.pi * 0.19 * t)) == pytest.approx(
        1 / 0.19, rel=0.02)
    assert math.isnan(M.dominant_period_s(t, np.ones_like(t)))


def test_stage_split_attributes_the_loss_to_the_right_stage():
    """THE diagnostic. Build a chain where ONLY the third stage (reference centroid ->
    actual centroid, i.e. the tracker) loses radius and adds lag, and check the split
    puts it there and nowhere else -- which is how the real lag was root-caused."""
    dt = 0.02
    t = np.arange(0, 30, dt)
    w = 2 * np.pi * 0.19
    def circ(r, lag):
        return np.column_stack([r * np.cos(w * (t - lag)), r * np.sin(w * (t - lag))])
    desired = circ(0.5, 0.0)
    ref_c = circ(0.5, 0.0)          # network references follow the desired path exactly
    act_c = circ(0.375, 0.5)        # THE TRACKER: loses 25% radius and 0.5 s
    payload = circ(0.375, 0.5)      # payload follows the drones faithfully
    rows = M.stage_split(t, desired, ref_c, act_c, payload)
    by = {r['stage']: r for r in rows}
    assert by['desired -> ref_centroid']['radius_ratio'] == pytest.approx(1.0, abs=0.02)
    assert by['desired -> ref_centroid']['phase_lag_s'] == pytest.approx(0.0, abs=0.03)
    assert by['ref_centroid -> actual_centroid']['radius_ratio'] == pytest.approx(0.75, abs=0.02)
    assert by['ref_centroid -> actual_centroid']['phase_lag_s'] == pytest.approx(0.5, abs=0.03)
    assert by['actual_centroid -> payload']['radius_ratio'] == pytest.approx(1.0, abs=0.02)
    assert by['actual_centroid -> payload']['phase_lag_s'] == pytest.approx(0.0, abs=0.03)


# ── payload attitude ─────────────────────────────────────────────────────────

def test_load_tilt_of_a_known_roll_is_that_angle():
    """A payload rolled 30 deg reports 30 deg. The attach failure is stated in these
    units (10 -> 33 -> 72 deg), so an error here would rewrite the finding."""
    for deg in (0.0, 10.0, 33.0, 72.0, 90.0):
        a = math.radians(deg)
        q = np.array([[math.cos(a / 2), math.sin(a / 2), 0.0, 0.0]])
        assert float(M.load_tilt_deg(q)[0]) == pytest.approx(deg, abs=1e-6)


def test_tilt_peak_and_settled_are_reported_separately():
    """A payload that spikes to 40 deg and then settles at 5 must report BOTH. Either
    number alone tells a misleading story about a reconfiguration."""
    t = np.arange(0, 30, 0.1)
    ang = np.where((t > 10) & (t < 12), 40.0, 5.0)
    a = np.radians(ang)
    q = np.column_stack([np.cos(a / 2), np.sin(a / 2), np.zeros_like(a), np.zeros_like(a)])
    peak, settled = M.tilt_peak_settled(t, q, t_event=10.0)
    assert peak == pytest.approx(40.0, abs=1e-6)
    assert settled == pytest.approx(5.0, abs=1e-6)   # steady window starts at t=15


# ── load sharing ─────────────────────────────────────────────────────────────

def test_tension_share_of_an_even_fleet_is_equal_and_sums_to_one():
    tens = np.tile([2.0, 2.0, 2.0, 2.0], (50, 1))
    out = M.tension_share(tens)
    assert sum(out['fraction']) == pytest.approx(1.0)
    assert out['spread'] == pytest.approx(0.0, abs=1e-12)


def test_tension_share_detects_a_newcomer_carrying_its_15_percent():
    """The A3 success criterion is 'newcomer carrying >= 15% of total tension', read
    straight off this number."""
    tens = np.tile([3.0, 3.0, 3.0, 1.0], (50, 1))
    out = M.tension_share(tens)
    assert out['fraction'][3] == pytest.approx(0.1, abs=1e-9)
    assert out['fraction'][3] < 0.15                 # this fleet FAILS the criterion


# ── event response ───────────────────────────────────────────────────────────

def test_event_settling_time_requires_the_error_to_STAY_in_the_band():
    """A signal that dips into the band and then leaves it has NOT settled. This is
    exactly the 45 deg ring attach, which is briefly level before it capsizes; a
    first-crossing definition would score it as the fastest-settling run in the set."""
    t = np.arange(0, 30, 0.1)
    err = np.where(t < 10, 0.5, np.where(t < 14, 0.02, 0.9))   # dips, then diverges
    assert M.event_settling_time(t, err, t_event=10.0, band=0.1) == math.inf


def test_event_settling_time_of_a_genuine_settle():
    t = np.arange(0, 30, 0.1)
    err = np.where(t < 13.0, 0.5, 0.02)
    assert M.event_settling_time(t, err, 10.0, band=0.1) == pytest.approx(3.0, abs=0.15)


def test_event_peak_error_only_looks_after_the_event():
    """A big error BEFORE the event is not the event's transient."""
    t = np.arange(0, 30, 0.1)
    err = np.where(t < 5, 2.0, 0.3)
    assert M.event_peak_error(t, err, t_event=10.0, window_s=10.0) == pytest.approx(0.3)


# ── effort and health ────────────────────────────────────────────────────────

def test_control_effort_counts_saturation_per_drone():
    thr = np.zeros((100, 2))
    thr[:, 0] = 0.30
    thr[:50, 1] = 0.30
    thr[50:, 1] = 0.62                                # drone 1 saturated half the time
    out = M.control_effort(thr)
    assert out['mean'][0] == pytest.approx(0.30)
    assert out['saturated_fraction'][0] == pytest.approx(0.0)
    assert out['saturated_fraction'][1] == pytest.approx(0.5)


def test_solver_health_separates_scattered_failures_from_consecutive_ones():
    """20 scattered failures are harmless (the tracker holds the last command); 16 in a
    row disarm the drone. The same failure RATE, opposite consequences."""
    scattered = np.zeros(1000)
    scattered[::50] = 4                               # 20 failures, never adjacent
    a = M.solver_health(scattered)
    assert a['max_consecutive'] == 1 and not a['would_disarm']

    burst = np.zeros(1000)
    burst[100:120] = 4                                # 20 failures, all adjacent
    b = M.solver_health(burst)
    assert a['fail_fraction'] == pytest.approx(b['fail_fraction'])
    assert b['max_consecutive'] == 20 and b['would_disarm']


def test_cable_accel_gap_reports_the_documented_runaway_numbers():
    """Modelled 1.66 against measured 17.0 is the 2026-07-28 signature; the gap is 15.34
    and the peak measured is 17.0."""
    n = 50
    modelled = np.tile([0.0, 0.0, 1.66], (n, 1))
    measured = np.tile([0.0, 0.0, 17.0], (n, 1))
    out = M.cable_accel_gap(measured, modelled)
    assert out['peak_gap'] == pytest.approx(15.34, abs=1e-9)
    assert out['peak_measured'] == pytest.approx(17.0, abs=1e-9)


# ── conventions ──────────────────────────────────────────────────────────────

def test_steady_window_starts_five_seconds_after_the_event():
    t = np.arange(0, 30, 0.1)
    m = M.steady_window(t, t_event=10.0)
    assert t[m][0] == pytest.approx(15.0, abs=0.05)
    assert t[m][-1] == pytest.approx(t[-1])


def test_summarise_reports_median_and_full_range_not_a_bare_mean():
    """The §9.2 reporting convention. A mean over 5 repeats hides the one that diverged,
    and the one that diverged is usually the finding."""
    out = M.summarise([0.10, 0.11, 0.12, 0.13, 3.0])
    assert out['median'] == pytest.approx(0.12)
    assert out['max'] == pytest.approx(3.0)
    assert out['n'] == 5


def test_summarise_ignores_nans_but_reports_the_count():
    out = M.summarise([0.1, math.nan, 0.3])
    assert out['n'] == 2 and out['median'] == pytest.approx(0.2)


# ── reconfiguration feasibility ──────────────────────────────────────────────

def test_cog_margin_matches_the_closed_form_for_a_symmetric_ring():
    """Removing one cable from a symmetric n-ring leaves a largest angular gap of
    4*pi/n, so the load's centre of mass stays inside the surviving attach polygon --
    the condition for it to hang level at all -- only for n >= 5. This is the result
    that explains the persistent tilt after a 4->3 detach without appealing to any
    controller tuning."""
    def ring(n, r=0.08):
        return [[r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n),
                 0.025] for k in range(n)]
    for n in (4, 5, 6, 7):
        _, gap, ok = M.cog_margin(ring(n)[1:])          # drop one cable
        assert gap == pytest.approx(math.degrees(4 * math.pi / n), abs=1e-9)
        assert ok == (n > 4)
    assert M.cog_margin(ring(4)[1:])[1] == pytest.approx(180.0, abs=1e-9)


def test_cog_margin_sign_says_inside_or_outside():
    """Negative margin means the centre of mass is on or outside the polygon, i.e. the
    load cannot hang level however the tensions are chosen."""
    def ring(n, r=0.08):
        return [[r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n),
                 0.0] for k in range(n)]
    assert M.cog_margin(ring(6))[0] > 0
    assert M.cog_margin(ring(6)[1:])[0] > 0             # 5 of 6 -> still inside
    assert M.cog_margin(ring(4)[1:])[0] <= 0            # 3 of 4 -> on the boundary
    assert M.cog_margin(ring(3)[1:])[0] < 0             # 2 cables -> no polygon
