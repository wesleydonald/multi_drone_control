"""The tilt cut card's harness pieces (card 2026-10-11_quad4_tilt_cut): MOCAP_TILT's argument, and the two criteria,
max_armed_tilt_deg (armed drones only, three rows, a MOCAP_TILT's own window excluded for that drone: run.csv shows
the fake attitude there) and disarmed_after_landed_s. Both off unless a config sets them."""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from experiment.config import Criteria, Event, mocap_tilt_arg, wrench_arg  # noqa: E402
import run_experiment as R  # noqa: E402


def _rows(n=4, t_end=10.0, dt=0.02, tilt=None, armed=None):
    rows = []
    k = 0
    while k * dt <= t_end + 1e-9:
        t = k * dt
        r = {'t': t}
        for i in range(n):
            r[f'd{i}_tilt_deg'] = (tilt or {}).get(i, lambda t: 2.0)(t)
            r[f'd{i}_armed'] = int((armed or {}).get(i, lambda t: True)(t))
        rows.append(r)
        k += 1
    return rows


CFG = types.SimpleNamespace(n_total=4)


def _check(c, rows, events):
    out = R.tilt_cut_checks(CFG, c, rows, events)
    return {name: (ok, detail) for name, ok, detail in out}


def test_mocap_tilt_arg():
    assert mocap_tilt_arg('3 90 2') == (3, 90.0, 2.0, 0.0)          # one step: a mis-solve
    assert mocap_tilt_arg('3 90') == (3, 90.0, 1.0, 0.0)
    assert mocap_tilt_arg('3 90 2 0.3') == (3, 90.0, 2.0, 0.3)      # a body tipping over 0.3 s
    for bad in ('3', '3 0 1', '3 90 0', '3 200 1', '3 90 2 -0.1', '3 90 2 0.3 1'):
        with pytest.raises(ValueError):
            mocap_tilt_arg(bad)
    Event(5.0, 'MOCAP_TILT', '3 90 0.02')
    with pytest.raises(ValueError):
        Event(5.0, 'MOCAP_TILT', 'x')


def test_both_criteria_are_off_by_default():
    c = Criteria()
    assert c.max_armed_tilt_deg is None and c.disarmed_after_landed_s is None
    assert R.tilt_cut_checks(CFG, c, _rows(), []) == []


def test_an_armed_drone_over_the_bar_fails_a_disarmed_one_does_not():
    c = Criteria(max_armed_tilt_deg=60)
    tipped = {3: lambda t: 128.0 if t >= 5.0 else 2.0}
    ok, detail = _check(c, _rows(tilt=tipped), [])['no armed drone over the tilt bar']
    assert not ok and 'drone 4' in detail and 't=5.0' in detail
    cut = {3: lambda t: t < 5.04}                              # cut 2 rows after the tip-over: under 3 rows
    assert _check(c, _rows(tilt=tipped, armed=cut), [])['no armed drone over the tilt bar'][0]


def test_a_mocap_tilt_window_is_excluded_for_its_drone_only():
    c = Criteria(max_armed_tilt_deg=60)
    fake = {3: lambda t: 90.0 if 5.0 <= t < 7.0 else 2.0}
    ev = [(5.0, 'MOCAP_TILT', '3 90 2')]
    assert _check(c, _rows(tilt=fake), ev)['no armed drone over the tilt bar'][0]
    carrier = {0: lambda t: 90.0 if 5.0 <= t < 7.0 else 2.0}  # the window is drone 4's, not a carrier's
    assert not _check(c, _rows(tilt=carrier), ev)['no armed drone over the tilt bar'][0]
    ramped = {3: lambda t: 90.0 if 5.2 <= t < 7.3 else 2.0}    # a 0.3 s ramp: the window is ramp + hold
    ev = [(5.0, 'MOCAP_TILT', '3 90 2 0.3')]
    assert _check(c, _rows(tilt=ramped), ev)['no armed drone over the tilt bar'][0]


def test_still_armed_after_landed():
    c = Criteria(disarmed_after_landed_s=5)
    ev = [(4.0, 'LANDED', '/fleet/landed')]
    armed = {i: (lambda t: t < 4.5) for i in range(3)}
    armed[3] = lambda t: True                                  # R1503: QUAD4 on his stream after LANDED
    ok, detail = _check(c, _rows(armed=armed), ev)['every drone disarmed after LANDED']
    assert not ok and '[4]' in detail
    armed[3] = lambda t: t < 4.5
    assert _check(c, _rows(armed=armed), ev)['every drone disarmed after LANDED'] == (True, 'all disarmed')
    assert _check(c, _rows(armed=armed), [])['every drone disarmed after LANDED'][0]   # no LANDED: not asserted


def test_wrench_takes_an_optional_torque_and_link():
    """Critic round 2, M-D: T6 is a body torque on QUAD4 (one motor lost at hover), not a sideways push."""
    ring = 'lift_system::payload::body'
    assert wrench_arg('0 10 0') == ((0.0, 10.0, 0.0), (0.0, 0.0, 0.0), ring)
    assert wrench_arg('0 10 0 x3_drone3::base_link') == ((0.0, 10.0, 0.0), (0.0, 0.0, 0.0), 'x3_drone3::base_link')
    assert wrench_arg('0 0 0 0.15 0 0 x3_drone3::base_link') == (
        (0.0, 0.0, 0.0), (0.15, 0.0, 0.0), 'x3_drone3::base_link')
    assert wrench_arg('0 0 0 0.15 0 0') == ((0.0, 0.0, 0.0), (0.15, 0.0, 0.0), ring)
    for bad in ('0 0', '0 0 0 1 2', '0 0 0 1 2 3 link extra'):
        with pytest.raises(ValueError):
            wrench_arg(bad)
