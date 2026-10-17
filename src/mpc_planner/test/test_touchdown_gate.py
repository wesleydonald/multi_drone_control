"""LAND touchdown re-arm and height gate (card 2026-10-17_touchdown_height_gate): the stall counts only with every
survivor within gate_dz of its rest height; off (rest None) is today's detector; R2032's false touchdown (the fleet
disarmed 0.5 m up) does not fire once the detector is re-armed at ring touchdown."""
import csv
import os

import numpy as np
import pytest

from mpc_planner.planner_node import TouchdownDetector, LAND_STALL_S

HZ, LAND_VEL = 10.0, 0.2
DT = 1.0 / HZ


def _settle(det, z0, z_stop, seconds, rest=None, gate=0.30):
    """Descend every drone at land_vel from z0 to z_stop, then hold; first time the detector fires, or None."""
    zs = list(z0)
    for k in range(int(seconds * HZ)):
        zs = [max(s, z - LAND_VEL * DT) for z, s in zip(zs, z_stop)]
        if det.update(zs, rest, gate):
            return (k + 1) * DT
    return None


def test_off_is_todays_detector():
    a, b = TouchdownDetector(LAND_VEL, HZ), TouchdownDetector(LAND_VEL, HZ)
    zs = [1.0, 1.0, 1.0]
    for k in range(120):
        zs = [max(0.1, z - LAND_VEL * DT) for z in zs]
        assert a.update(zs) == b.update(zs, None, 0.30)


def test_stall_high_above_rest_does_not_fire():
    det = TouchdownDetector(LAND_VEL, HZ)
    assert _settle(det, [1.0] * 3, [0.50, 0.10, 0.10], 15.0, rest=[0.10] * 3) is None


def test_stall_within_the_gate_fires():
    det = TouchdownDetector(LAND_VEL, HZ)
    t = _settle(det, [1.0] * 3, [0.35, 0.10, 0.10], 15.0, rest=[0.10] * 3)
    assert t is not None and t <= (1.0 - 0.10) / LAND_VEL + LAND_STALL_S + 3 * DT


def test_a_drone_without_a_rest_record_is_not_gated():
    det = TouchdownDetector(LAND_VEL, HZ)
    assert _settle(det, [1.0] * 3, [0.50, 0.10, 0.10], 15.0, rest=[None, 0.10, 0.10]) is not None


def test_gate_is_one_sided():
    """A drone that lands below its rest (it took off from a raised mat) is not held by the gate."""
    det = TouchdownDetector(LAND_VEL, HZ)
    assert _settle(det, [1.0] * 3, [0.10, 0.10, 0.10], 15.0, rest=[0.40, 0.10, 0.10]) is not None


def _r2032():
    d = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'results', '2026-10-16')
    hits = [os.path.join(d, x) for x in os.listdir(d) if x.startswith('R2032_')] if os.path.isdir(d) else []
    if not hits:
        pytest.skip('R2032 log not present')
    rows = list(csv.DictReader(open(os.path.join(hits[0], 'logs', 'run.csv'))))
    ev = {r['event']: float(r['sim_time']) for r in csv.DictReader(open(os.path.join(hits[0], 'logs', 'events.csv')))}
    t = np.array([float(r['t']) for r in rows])
    zs = np.array([[float(r[f'd{i}_z']) for i in range(3)] for r in rows])
    ring = np.array([float(r['payload_z']) for r in rows])
    return t, zs, ring, ev['LAND'], ev['LANDED']


def test_replay_r2032_fires_today_and_not_once_rearmed():
    """Today's detector, armed at LAND, fires within the recorded run before LANDED went out (64.70 s). Re-armed at
    ring touchdown, it does not fire on the same heights before that time."""
    t, zs, ring, t_land, t_landed = _r2032()
    t_down = float(t[np.argmax((t > t_land) & (ring <= ring.min() + 0.005))])
    today, rearmed = TouchdownDetector(LAND_VEL, HZ), TouchdownDetector(LAND_VEL, HZ)
    fired_today = fired_rearmed = None
    for k in range(int((t_landed - t_land) * HZ) + 2):
        tk = t_land + (k + 1) * DT
        z = [float(np.interp(tk, t, zs[:, i])) for i in range(3)]
        if today.update(z) and fired_today is None:
            fired_today = tk
        if abs(tk - t_down) < DT / 2:
            rearmed.reset()
        if rearmed.update(z) and fired_rearmed is None:
            fired_rearmed = tk
    assert fired_today is not None and t_down < fired_today <= t_landed + DT
    assert fired_rearmed is None
