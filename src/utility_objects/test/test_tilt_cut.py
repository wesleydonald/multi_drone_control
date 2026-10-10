"""The QUAD4 tilt-cut rule (card 2026-10-11_quad4_tilt_cut): 60 deg for 3 poses, from TAKEOFF to the next ARM,
whatever the tracker's armed flag; a single-pose attitude step over 45 deg is a mocap glitch, not counted, with a
hold-off; the optional slow tier."""
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


def test_three_poses_over_60_deg_latch_once():
    r = _flying()
    out = _feed(r, [40, 50, 59, 61, 62, 63, 64, 65])
    assert out == [None] * 5 + ['tilt', None, None]
    assert r.fault == 'tilt' and r.fault_tilt == pytest.approx(63.0, abs=1e-6)


def test_a_dip_back_under_resets_the_count():
    r = _flying()
    assert _feed(r, [58, 61, 62, 59, 61, 62]) == [None] * 6
    assert r.fault is None


def test_nothing_before_takeoff_and_the_flight_latch_outlives_a_disarm():
    """The latch is the rule's own (critic round 1, C1): set at TAKEOFF, cleared only at the next ARM."""
    r = TiltCutRule()
    assert _feed(r, [90] * 10) == [None] * 10                  # on its mat, before TAKEOFF
    r.takeoff()
    assert 'tilt' in _feed(r, [90] * 5, t0=1.0)                # after a disarm the caller simply keeps feeding
    r.arm()
    assert not r.flying and r.fault is None and r.max_tilt == 0.0


def test_a_single_pose_step_over_45_deg_is_a_glitch_with_a_hold_off():
    r = _flying()
    out = _feed(r, [5, 90, 90, 90, 90])                        # a mis-solve that sticks for 4 poses (33 ms)
    assert out == [None] * 5 and r.glitches == 1
    # after the 0.5 s hold-off a mis-solve that is still there counts: the attitude is wrong
    out = [r.step(0.6 + k / HZ, _q(90)) for k in range(4)]
    assert out[2] == 'tilt'


def test_a_real_tumble_is_continuous_and_cut():
    """A knock: 400 deg/s is 3.3 deg per pose at 120 Hz, never a glitch (R1263: 169 deg within ~0.5 s)."""
    r = _flying()
    degs = [3.3 * k for k in range(40)]
    out = _feed(r, degs)
    assert r.glitches == 0 and out.index('tilt') == 21         # 69 deg, the third pose over 60


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
    for kw in ({'cut_deg': 0.0}, {'samples': 0}, {'glitch_step_deg': 0.0}, {'slow_deg': -1.0}):
        with pytest.raises(ValueError):
            TiltCutConfig(**kw)
