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


def test_steady_window_stops_at_land():
    """A hover-then-LAND run must not average the descent into the settled value."""
    t = np.arange(0.0, 20.0, 0.02)
    m = M.steady_window(t, t_event=6.0, t_end=16.0)
    assert t[m].min() >= 11.0 - 1e-9 and t[m].max() < 16.0
    m2 = M.steady_window(t, t_event=6.0)
    assert t[m2].max() > 19.9                       # no cutoff without a LAND


# ── per-event windows, M1 stages, M2 summary (step 2a) ──────────────────────

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fixtures')


def _flight(n=3, dt=0.02, T=30.0):
    """A level hover: ring at 0.6 on its reference, carriers at 1.0 on theirs."""
    t = np.arange(0.0, T, dt)
    d = {'t': t, 'payload_tilt_deg': np.full(t.size, 0.5),
         'payload_z': np.full(t.size, 0.6), 'payload_ref_z': np.full(t.size, 0.6)}
    for i in range(n):
        d[f'd{i}_z'] = np.full(t.size, 1.0)
        d[f'd{i}_ref_z'] = np.full(t.size, 1.0)
        d[f'd{i}_tilt_deg'] = np.full(t.size, 3.0)
    return t, d


def test_event_window_step_gives_peak_and_settle():
    """A 6 deg step at the event that falls to 1 deg 3 s later: peak 6, settled after
    exactly 3 s (under 2 deg and it stays there for more than 1 s)."""
    t, d = _flight()
    d['payload_tilt_deg'][(t >= 10.0) & (t < 13.0)] = 6.0
    d['payload_tilt_deg'][t >= 13.0] = 1.0
    w = M.event_window_metrics(t, d, 10.0)
    assert w['ring_tilt_peak_deg'] == pytest.approx(6.0)
    assert w['settle_s'] == pytest.approx(3.0, abs=0.021)
    assert w['settle_censored'] is False
    assert w['t_end'] == pytest.approx(25.0)                 # +15 s with no other event


def test_settle_is_searched_from_the_peak_not_from_the_event():
    """Level for 0.8 s after the event, then a slow build to 5 deg: the quiet start is
    not a settle."""
    t, d = _flight()
    tilt = d['payload_tilt_deg']
    tilt[(t >= 10.8) & (t < 14.0)] = 5.0
    tilt[t >= 14.0] = 1.0
    w = M.event_window_metrics(t, d, 10.0)
    assert w['settle_s'] == pytest.approx(4.0, abs=0.021)


def test_settle_is_zero_when_the_tilt_never_leaves_the_band():
    """A quiet event (1.5 deg peak, late in the window) settled at once, not at its peak."""
    t, d = _flight()
    d['payload_tilt_deg'][(t >= 18.0) & (t < 18.5)] = 1.5
    w = M.event_window_metrics(t, d, 10.0)
    assert w['settle_s'] == 0.0 and w['settle_censored'] is False


def test_settle_is_censored_when_the_window_ends_first():
    """Still at 6 deg when LAND cuts the window: no settle number, marked censored."""
    t, d = _flight()
    d['payload_tilt_deg'][t >= 10.0] = 6.0
    w = M.event_window_metrics(t, d, 10.0, t_end=16.0)
    assert math.isnan(w['settle_s']) and w['settle_censored'] is True
    assert w['window_s'] == pytest.approx(6.0)


def test_carrier_dip_excludes_the_event_drone_and_splits_actual_from_reference():
    """Carrier 0 sags 5 cm (its reference only 2 cm); the newcomer (drone 2) drops 30 cm
    and is not a carrier."""
    t, d = _flight()
    sag = (t >= 11.0) & (t < 12.0)
    d['d0_z'][sag] = 0.95
    d['d0_ref_z'][sag] = 0.98
    d['d1_z'][sag] = 0.99
    d['d2_z'][sag] = 0.70
    w = M.event_window_metrics(t, d, 10.0, exclude=(2,))
    assert w['carrier_dip_m'] == pytest.approx(0.05) and w['carrier_dip_drone'] == 0
    assert w['carrier_dip_ref_m'] == pytest.approx(0.02) and w['carrier_dip_ref_drone'] == 0


def test_ring_z_error_is_the_last_second_of_the_window():
    t, d = _flight()
    d['payload_z'][t >= 15.0] = 0.65
    w = M.event_window_metrics(t, d, 10.0, t_end=16.0)
    assert w['ring_z_err_last1s_m'] == pytest.approx(0.05)


def test_window_runs_to_the_next_disturbing_event_only():
    """RELEASED / WAIT_* / LIFTED do not cut a window; the FOLD_IN that follows its own
    weld within 2 s is the same disturbance; LAND cuts it."""
    ev = [(20.0, 'REWELD', ''), (20.1, 'FOLD_IN', '3'), (21.0, 'RELEASED', ''),
          (22.0, 'WAIT_REWELD', ''), (23.0, 'LIFTED', ''), (26.0, 'LAND', '')]
    assert M.window_end(ev, 20.0, 'REWELD') == pytest.approx(26.0)
    assert M.window_end(ev[:-1], 20.0, 'REWELD') == pytest.approx(35.0)
    assert M.window_end([(5.0, 'DETACH', '3'), (7.0, 'PHASE:DROP_OBJECT', '')],
                        5.0, 'DETACH') == pytest.approx(7.0)


def test_mask_released_blanks_the_partner_between_release_and_reweld():
    t, d = _flight(n=4)
    d['d3_tilt_deg'][:] = 40.0
    ev = [(5.0, 'RELEASED', ''), (15.0, 'REWELD', '')]
    m = M.mask_released(d, ev, 3)
    gone = (t >= 5.0) & (t < 15.0)
    assert np.all(np.isnan(m['d3_tilt_deg'][gone]))
    assert np.all(m['d3_tilt_deg'][~gone] == 40.0)
    assert np.all(d['d3_tilt_deg'] == 40.0)                     # input untouched


def test_ball_in_ring_frame_follows_the_ring_yaw():
    """Ring at (1, 0, 0.5) yawed 90 deg; ball 0.1 m along world +y and 5 cm up is 0.1 m
    along the ring's +x: rho 0.1, z_rel 0.05, in the net."""
    c = math.cos(math.pi / 4)
    d = {'payload_x': np.array([1.0]), 'payload_y': np.array([0.0]),
         'payload_z': np.array([0.5]), 'payload_qw': np.array([c]),
         'payload_qx': np.array([0.0]), 'payload_qy': np.array([0.0]),
         'payload_qz': np.array([c]), 'ball_x': np.array([1.0]),
         'ball_y': np.array([0.1]), 'ball_z': np.array([0.55])}
    rho, z_rel = M.ball_in_ring_frame(d)
    assert rho[0] == pytest.approx(0.1) and z_rel[0] == pytest.approx(0.05)
    s = M.ball_state(rho[0], z_rel[0])
    assert s['in_net'] and s['on_footprint']
    assert not M.ball_state(0.1, 0.30)['in_net']                   # too high: not in it
    assert M.ball_state(0.1, 0.30, rho_only=True)['in_net']        # LANDED: rho only


def test_m1_counts_reattach_retries_only_before_the_first_handoff():
    t, d = _flight(n=4)
    ev = [(1.0, 'PHASE:APPROACH_ABOVE_TARGET', ''), (2.0, 'PHASE:MATCH_VELOCITY', ''),
          (3.0, 'PHASE:APPROACH_ABOVE_TARGET', ''), (4.0, 'PHASE:APPROACH_ABOVE_TARGET', ''),
          (5.0, 'HANDOFF', ''), (6.0, 'PHASE:APPROACH_ABOVE_TARGET', '')]
    log = ('Planner phase: X -> APPROACH_ABOVE_TARGET. Reason\n'
           'Planner phase: APPROACH_ABOVE_TARGET -> TAKEOFF. Reason\n')
    m = M.m1_stage_metrics(t, d, ev, phase_log=log)
    assert m['reattach_retries'] == 2
    assert m['phases_missed'] == ['TAKEOFF']
    assert 'PICKUP' in m['missing'] and 'HANDOFF' not in m['missing']


def test_r0646_fixture_reweld_and_detach_windows():
    """R0646 (M1, trimmed): detach peak 5.41 deg, reweld peak 10.43 deg with carrier 1
    dipping 6.4 cm (6.3 cm on its reference). The reweld window ends at LAND 6 s later,
    so its settle is censored: the planning pass's 9.39 s read through LAND into the
    touchdown, where the ring is level because it is on the floor."""
    run = os.path.join(FIXTURES, 'R0646_trim')
    d = M.read_csv(os.path.join(run, 'logs', 'run.csv'))
    m = M.m1_stage_metrics(d['t'], d, M.read_event_list(run))
    w = {x['event']: x for x in m['windows']}
    assert w['DETACH']['ring_tilt_peak_deg'] == pytest.approx(5.41, abs=0.01)
    assert w['REWELD']['ring_tilt_peak_deg'] == pytest.approx(10.43, abs=0.01)
    assert w['REWELD']['carrier_dip_drone'] == 1
    assert w['REWELD']['carrier_dip_m'] == pytest.approx(0.0645, abs=0.001)
    assert w['REWELD']['carrier_dip_ref_m'] == pytest.approx(0.0634, abs=0.001)
    assert w['REWELD']['window_s'] == pytest.approx(6.0)
    assert w['REWELD']['settle_censored'] is True
    assert m['stage_t']['REWELD'] == pytest.approx(107.838)


def test_reweld_is_a_primary_event():
    assert M.primary_event({'WELD': math.nan, 'REWELD': 107.8, 'DETACH': 18.5}) == \
        ('REWELD', 107.8)


def test_m2_counts_fallback_installs_not_status_lines(tmp_path):
    """His backend prints fallback=true on every status tick while a fallback is active;
    only the C1F.4 ERROR line marks an install. The join is the most drones ever attached."""
    status = ("[b-1] execution tracking ... fallback=true certified=true mode='HOVER' "
              "reason='TRACKING_TUBE_VIOLATION pos=[0 0 0]' count/recovered=1/0\n")
    rt = ("M2D state=M2D_SUCCESS_HOLD active=None attached=['drone_0', 'drone_1', "
          "'drone_2', 'drone_3'] landing=none\n"
          "[b-1] [ERROR] [1.0] [b]: C1F.4 execution certificate invalid "
          "(TRACKING_TUBE_VIOLATION pos=[0.1 0 0] vel_norm=0.2). Installed HOVER fallback; "
          "certified=true.\n" + status * 30 +
          "[b-2] [ERROR] [2.0] [b]: C1F.4 failed to install execution fallback: boom\n"
          "M2D state=ABORT active=None attached=[] landing=none\n")
    (tmp_path / 'ev').mkdir()
    (tmp_path / 'ev' / 'runtime.log').write_text(rt)
    (tmp_path / 'runner.log').write_text('Evidence:   ev\n')
    s = M.summarise_m2(str(tmp_path))
    assert s['fallback_count'] == 2 and s['tube_violation_count'] == 1
    assert s['join_attached'] == 4


def _check_t0015(s):
    assert s['join_attached'] == 4
    assert s['fallback_count'] == 0 and s['tube_violation_count'] == 0
    assert s['envelope_faults'] == 0
    assert s['max_ref_age_s'] == pytest.approx(0.92)
    assert s['landed'] is True
    assert s['ring_tilt_peak_deg'] == pytest.approx(1.78)
    assert s['ring_tilt']['lift']['peak_deg'] == pytest.approx(1.78)
    assert s['load_z_min_m'] == pytest.approx(0.6026, abs=1e-4)
    assert s['load_z_max_m'] == pytest.approx(0.6050, abs=1e-4)
    assert s['t']['touchdown'] == pytest.approx(119.9)
    assert s['drone_tilt_ok'] is True


def test_t0015_m2_fixture():
    """T0015 (M2, trimmed): join 4/4, no fallback / tube violation / fault, ring tilt
    peak 1.78 deg (in the lift), load_z 0.6026-0.6050 m. The window is only 0.8 s long:
    T0015's hover was ~6 s of sim (spin(hover_s*4) wall), which the driver now counts in
    sim seconds."""
    s = M.summarise_m2(os.path.join(FIXTURES, 'T0015_m2'))
    _check_t0015(s)
    assert max(s['drone_tilt_peak_deg'].values()) == pytest.approx(15.53, abs=0.01)


def test_t0015_m2_legacy_layout_from_the_real_run():
    """The real T0015 dir: its controller logs are found through ours.log, not MDC_RUN_DIR."""
    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    run = os.path.join(repo, 'results', '2026-09-26', 'T0015_m2_handover_token')
    if not os.path.isdir(run):
        pytest.skip('T0015 not on this machine')
    s = M.summarise_m2(run)
    if s.get('dissipative_log') is None or s.get('runtime_log') is None \
            or not os.path.exists(s['runtime_log']):
        pytest.skip('T0015 controller / evidence logs not on this machine')
    _check_t0015(s)
    assert len(s['tracker_logs']) == 4
