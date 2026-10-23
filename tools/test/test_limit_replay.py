"""tools/limit_replay.py (LIM-0, critic M4): the fleet manager's position-limit test over a flight's run.csv, the quads'
offsets from the ring scaled from the flight's reach (its params.json) to the 1 m formation's, until the first release;
a trip needs POS_LIMIT_SAMPLES past the limit in a row, airborne and between TAKEOFF and DISARM."""
import csv
import json
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import limit_replay as lr  # noqa: E402


def _flight(tmp_path, ring_x, quad_z=1.0, events=(('TAKEOFF', 0.5), ('DISARM', 100.0)), cable=0.55):
    """A one-quad flight: the ring at ring_x[k] (y 0), the quad 0.5 m further out in x (the 0.55 m reach at 60 deg)."""
    day = tmp_path / '2026-10-22'
    run = day / 'f1_run' / 'logs'
    run.mkdir(parents=True)
    (day / 'f1.log').write_text('')
    pj = day / 'f1_logs' / 'logs' / 'dissipative_planner' / '20261022_090000'
    pj.mkdir(parents=True)
    (pj / 'params.json').write_text(json.dumps({'cable_len': cable, 'attach_radius': 0.225, 'cable_elev_deg': 60.0}))
    with open(run / 'run.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t', 'payload_x', 'payload_y', 'payload_z', 'payload_ref_x', 'payload_ref_y', 'payload_ref_z',
                    'd0_x', 'd0_y', 'd0_z'])
        for k, x in enumerate(ring_x):
            z = 0.05 if k == 0 else quad_z
            w.writerow([0.02 * k, x, 0.0, 0.5, x, 0.0, 0.5 if k else 0.0, x + 0.5, 0.0, z])
    with open(run / 'events.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['sim_time', 'event', 'arg'])
        for ev, t in events:
            w.writerow([t, ev, ''])
    return str(day / 'f1'), str(run / 'run.csv')


def test_three_scaled_samples_past_the_limit_trip_and_two_do_not(tmp_path):
    # 0.5 m out scales to 0.725: a ring at 1.04 puts the quad at 1.765 at 1 m (1.54 as flown)
    f, run = _flight(tmp_path, [0.0] * 60 + [1.04] * 3 + [0.0] * 5)
    row = lr.replay(f, run, 1.75, lr.REACH_1M)
    assert row['reach'] == pytest.approx(0.5) and row['scaled'] == pytest.approx(1.765)
    assert row['trip_flown'] is None and row['trip_scaled'] == (pytest.approx(0.02 * 62), 0)
    assert row['m_land'] == pytest.approx(-0.015) and row['m_clamp'] == pytest.approx(-0.065)
    f2, run2 = _flight(tmp_path / 'b', [0.0] * 60 + [1.04] * 2 + [0.0] * 5)
    assert lr.replay(f2, run2, 1.75, lr.REACH_1M)['trip_scaled'] is None


def test_on_the_floor_before_takeoff_or_after_a_release_nothing_is_scaled_or_trips(tmp_path):
    f, run = _flight(tmp_path, [0.0] + [1.04] * 10, quad_z=0.1)                          # never airborne
    assert lr.replay(f, run, 1.75, lr.REACH_1M)['trip_scaled'] is None
    f, run = _flight(tmp_path / 'b', [0.0] + [1.04] * 10, events=(('TAKEOFF', 0.5), ('DISARM', 100.0)))
    assert lr.replay(f, run, 1.75, lr.REACH_1M)['trip_scaled'] is None                   # TAKEOFF after these
    f, run = _flight(tmp_path / 'c', [0.0] * 30 + [1.04] * 10,
                     events=(('TAKEOFF', 0.0), ('DROP', 0.1), ('DISARM', 100.0)))
    row = lr.replay(f, run, 1.75, lr.REACH_1M)
    assert row['scaled'] == pytest.approx(1.54) and row['trip_scaled'] is None           # freed: as flown


def test_a_1m_flight_is_not_scaled_and_a_path_scale_shrinks_the_ring_about_its_hover_point(tmp_path):
    f, run = _flight(tmp_path, [0.0] * 30 + [1.04] * 5, cable=1.0)
    row = lr.replay(f, run, 1.75, lr.REACH_1M)
    assert row['reach'] == pytest.approx(lr.REACH_1M) and row['scaled'] == pytest.approx(1.54)
    ref = np.array([[0.1, 0.0, 0.0], [0.1, 0.0, 1.0], [1.1, 0.5, 1.0]])
    assert np.allclose(lr.path_shift(ref, 0.75), [[0.0, 0.0], [0.0, 0.0], [-0.25, -0.125]])
    assert np.allclose(lr.path_shift(ref, 1.0), 0.0)
