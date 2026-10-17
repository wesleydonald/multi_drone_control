"""LAND touchdown re-arm (card 2026-10-17_touchdown_height_gate; the height gate, arm B, was deleted 17 Oct: it never
acted): R2032's false touchdown (the fleet disarmed 0.5 m up) does not fire once the detector is re-armed at ring
touchdown."""
import csv
import os

import numpy as np
import pytest

from mpc_planner.planner_node import TouchdownDetector

HZ, LAND_VEL = 10.0, 0.2
DT = 1.0 / HZ


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
