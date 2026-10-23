"""tools/proof_replay.py catch_report: the card's bars from a bag, on synthetic tracks (no bag, no ROS)."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import proof_replay as pr  # noqa: E402

WELD = (0.0, 0.2, 0.02)
L = 0.57


def _data(tip_rel, body_dz, plate_dz=lambda t: 0.0, t1=10.0, phases=(), magnet=(), mux=()):
    """100 Hz: a level ring at the origin; the tip at WELD + tip_rel(t); QUAD4's body L above the tip + body_dz(t)."""
    t = np.arange(0.0, t1, 0.01)
    ring = np.array([(s, 0.0, 0.0, 1.0 + plate_dz(s), 0.0, 0.0, 0.0, 1.0) for s in t])
    tip = np.array([(s, *(np.array(WELD) + [0.0, 0.0, 1.0 + plate_dz(s)] + tip_rel(s))) for s in t])
    body = np.array([(s, tip[k, 1], tip[k, 2], tip[k, 3] + L + body_dz(s)) for k, s in enumerate(t)])
    return {'tip': tip, 'ring': ring, 'body': body, 'lift': [], 'phases': list(phases), 'magnet': list(magnet),
            'mux': list(mux)}


def test_a_held_catch_on_a_taut_tether_is_a_catch():
    d = _data(lambda t: np.zeros(3), lambda t: 0.0, plate_dz=lambda t: 0.01 * min(t, 2.0))
    ev = [(1.0, 'contact', {'l_hang_m': L}), (3.0, 'proven', {})]
    line = pr.catch_report(ev, d, WELD)[0]
    assert 'CATCH, proven' in line and 'BAR' not in line


def test_a_resting_tip_on_a_slack_tether_is_no_catch_and_a_proof_of_it_is_flagged():
    d = _data(lambda t: np.zeros(3), lambda t: -0.02)            # 2 cm of slack: never taut
    line = pr.catch_report([(1.0, 'contact', {'l_hang_m': L})], d, WELD)[0]
    assert 'no catch, not proven' in line
    line = pr.catch_report([(1.0, 'contact', {'l_hang_m': L}), (3.0, 'proven', {})], d, WELD)[0]
    assert 'BAR 2 MISMATCH' in line


def test_a_tip_that_leaves_after_the_proof_fails_bar_1():
    d = _data(lambda t: np.array([0.0, 0.0, 0.03 if t > 4.0 else 0.0]), lambda t: 0.0)
    line = pr.catch_report([(1.0, 'contact', {'l_hang_m': L}), (3.0, 'proven', {})], d, WELD)[0]
    assert 'BAR 1 FAIL' in line


def test_a_climb_that_lifts_the_plate_fails_bar_3():
    lift = lambda t: max(0.0, min(0.05, (t - 2.0) * 0.05))
    d = _data(lambda t: np.zeros(3), lambda t: 0.0, plate_dz=lift,
              phases=[(0.0, 'ATTACH_READY'), (2.0, 'APPROACH_ABOVE_TARGET')])
    lines = pr.catch_report([(0.5, 'contact', {'l_hang_m': L})], d, WELD)
    assert any('BAR 3 FAIL' in s for s in lines)
    free = _data(lambda t: np.array([0.0, 0.0, lift(t)]), lambda t: 0.0,
                 phases=[(0.0, 'ATTACH_READY'), (2.0, 'APPROACH_ABOVE_TARGET')])
    lines = pr.catch_report([(0.5, 'contact', {'l_hang_m': L})], free, WELD)
    assert any('climb back' in s and s.endswith('ok') for s in lines)
