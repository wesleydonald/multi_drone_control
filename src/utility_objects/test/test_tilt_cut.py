"""The QUAD4 tilt-cut rule (card 2026-10-11_quad4_tilt_cut): 60 deg held 25 ms, from TAKEOFF to the next ARM,
whatever the tracker's armed flag; an attitude change faster than 3000 deg/s between two poses is a mocap glitch, not
counted, with a hold-off (a rate, so a real tumble across a mocap gap is counted: critic round 2, M-A); the optional
slow tier."""
import math

import pytest

from utility_objects.tilt_cut import TiltCutConfig, TiltCutRule, attitude_step_deg

HZ = 120.0


def _q(deg, axis='x'):
    h = math.radians(deg) / 2.0
    return (math.cos(h), math.sin(h), 0.0, 0.0) if axis == 'x' else (math.cos(h), 0.0, 0.0, math.sin(h))


def _flying(**kw):
    r = TiltCutRule(TiltCutConfig(**kw))
    r.takeoff()
    return r


def _feed(r, degs, t0=0.0):
    return [r.step(t0 + k / HZ, _q(d)) for k, d in enumerate(degs)]


def test_60_deg_held_25_ms_latches_once():
    r = _flying()
    out = _feed(r, [40, 50, 59, 61, 62, 63, 64, 65, 66])
    assert out == [None] * 6 + ['tilt', None, None]            # 61 deg at 25 ms, three intervals later at 120 Hz
    assert r.fault == 'tilt' and r.fault_tilt == pytest.approx(64.0, abs=1e-6)


def test_a_dip_back_under_resets_the_count():
    r = _flying()
    assert _feed(r, [58, 61, 62, 63, 59, 61, 62, 63]) == [None] * 8
    assert r.fault is None


def test_nothing_before_takeoff_and_the_flight_latch_outlives_a_disarm():
    """The latch is the rule's own (critic round 1, C1): set at TAKEOFF, cleared only at the next ARM."""
    r = TiltCutRule()
    assert _feed(r, [90] * 10) == [None] * 10                  # on its mat, before TAKEOFF
    r.takeoff()
    assert 'tilt' in _feed(r, [90] * 5, t0=1.0)                # after a disarm the caller simply keeps feeding
    r.arm()
    assert not r.flying and r.fault is None and r.max_tilt == 0.0


def test_a_one_pose_jump_faster_than_3000_deg_s_is_a_glitch_with_a_hold_off():
    r = _flying()
    out = _feed(r, [5, 90, 90, 90, 90, 90])                    # 85 deg in 8 ms: a mis-solve that sticks 42 ms
    assert out == [None] * 6 and r.glitches == 1
    # after the 0.5 s hold-off a mis-solve that is still there counts: the attitude is wrong
    out = [r.step(0.6 + k / HZ, _q(90)) for k in range(5)]
    assert out[3] == 'tilt'


def test_a_real_tumble_is_continuous_and_cut():
    """A knock: 400 deg/s is 3.3 deg per pose at 120 Hz, never a glitch (R1263: 169 deg within ~0.5 s)."""
    r = _flying()
    degs = [3.3 * k for k in range(40)]
    out = _feed(r, degs)
    assert r.glitches == 0 and out.index('tilt') == 22         # 72.6 deg, 25 ms after the first pose over 60


@pytest.mark.parametrize('gap_s', [0.06, 0.15])
def test_a_tumble_across_a_mocap_gap_is_cut_within_0_1_s_of_the_gap_end(gap_s):
    """Critic round 2, M-A: 720 deg/s across a 60 ms (43 deg) or 150 ms (108 deg) gap. Judged by its rate the jump
    is a body, so the cut comes within 0.1 s of the first pose after the gap, not after a 0.5 s hold-off."""
    r = _flying()
    t, out, end = 0.0, None, None
    while t < 1.0 and out is None:
        out = r.step(t, _q(min(720.0 * t, 170.0)))
        if end is None and 720.0 * t >= 30.0:                  # the gap opens at 30 deg
            t += gap_s
            end = t
        else:
            t += 1.0 / HZ
    assert out == 'tilt' and r.glitches == 0 and t - 1.0 / HZ - end <= 0.1


def test_the_slow_tier_is_off_by_default_and_holds_30_deg_for_half_a_second():
    r = _flying()
    assert set(_feed(r, [35] * int(HZ))) == {None}             # default: no slow tier
    s = _flying(slow_deg=30.0, slow_s=0.5)
    out = _feed(s, [35] * int(HZ))
    assert out.index('slow') == int(0.5 * HZ)
    t = _flying(slow_deg=30.0, slow_s=0.5)
    assert set(_feed(t, ([35] * 30 + [25]) * 4)) == {None}     # never held for 0.5 s


def test_attitude_step_is_sign_invariant():
    q = _q(30.0)
    assert attitude_step_deg(q, tuple(-v for v in q)) == pytest.approx(0.0, abs=1e-6)
    assert attitude_step_deg(_q(10.0), _q(40.0)) == pytest.approx(30.0, abs=1e-6)
    assert attitude_step_deg(_q(0.0), _q(90.0, 'z')) == pytest.approx(90.0, abs=1e-6)   # yaw counts too


def test_bad_config_is_refused():
    for kw in ({'cut_deg': 0.0}, {'cut_hold_s': -0.1}, {'glitch_rate_dps': 0.0}, {'slow_deg': -1.0}):
        with pytest.raises(ValueError):
            TiltCutConfig(**kw)
