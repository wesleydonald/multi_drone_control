"""tools/attach_metrics.py on a synthetic twin attach run: a newcomer that climbs, hovers 0.5 m from a carrier,
steps out after LAND and is disarmed in the air at LANDED.

    python3 -m pytest tools/test/test_attach_metrics.py -q
"""
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import attach_metrics as am  # noqa: E402

ARM_EPOCH = 1791000000.0


def make_run(tmp_path):
    d = tmp_path / 'R9999_sim_gz_twin_r8a_dry'
    (d / 'logs').mkdir(parents=True)
    t = np.round(np.arange(0.0, 60.0, 0.05), 3)
    wall = 2.0 * t                                    # the sim at half real time
    rows = {'t': t, 'wall': wall, 'step': np.arange(len(t)),
            'payload_x': 0.0 * t, 'payload_y': 0.0 * t, 'payload_z': np.where(t > 10, 1.0, 0.02),
            'payload_tilt_deg': np.where((t > 20) & (t < 21), 4.0, 1.0)}
    for i, ang in enumerate((270, 30, 150)):
        a = np.radians(ang)
        rows.update({f'd{i}_x': 0.7 * np.cos(a) + 0 * t, f'd{i}_y': 0.7 * np.sin(a) + 0 * t,
                     f'd{i}_z': np.where(t > 10, 1.5, 0.1), f'd{i}_armed': np.ones_like(t)})
    # newcomer: on the floor at (0, 1.4), climbs at 15 s, hovers at (0, 0.95, 1.6) from 25 s,
    # steps out to r 1.3 after LAND (40 s), disarmed at 1.5 m at LANDED (50 s)
    nx, ny = 0 * t, np.where(t < 25, 1.4, np.where(t < 40, 0.95, np.minimum(0.95 + (t - 40) * 0.1, 1.3)))
    nz = np.where(t < 15, 0.1, np.where(t < 50, 1.6, np.where(t < 52, 1.5, 0.1)))
    rows.update({'d3_x': nx, 'd3_y': ny, 'd3_z': nz, 'd3_armed': np.where(t < 50, 1.0, 0.0)})
    pd.DataFrame(rows).to_csv(d / 'logs' / 'run.csv', index=False)
    pd.DataFrame({'sim_time': [3.0, 6.0, 8.0, 10.0, 15.0, 40.0, 50.0],
                  'event': ['ARM', 'TAKEOFF', 'WAIT_LIFT', 'LIFTED', 'MAGNET', 'LAND', 'LANDED'],
                  'arg': ['', '', '0.3 60', '2.0', 'ON', '', '/fleet/landed']}).to_csv(
        d / 'logs' / 'events.csv', index=False)
    stamp = lambda tt: f'{ARM_EPOCH + 2.0 * (tt - 3.0):.6f}'  # noqa: E731
    (d / 'logs' / 'launch.log').write_text('\n'.join([
        f"[fleet_manager-1] [INFO] [{stamp(3.0)}] [fleet_manager]: Fleet command received: 'ARM'",
        f"[dissipative_planner-2] [INFO] [{stamp(15.1)}] [dissipative_planner]: [dissipative] approach: drone 3 climb",
        f"[dissipative_planner-2] [INFO] [{stamp(21.0)}] [dissipative_planner]: [dissipative] approach: drone 3 transit",
    ]) + '\n')
    (d / 'manifest.json').write_text(json.dumps({'launch_args': {'num_drones': '3'}}))
    return str(d)


def test_scores_the_newcomer(tmp_path):
    r = am.analyse(make_run(tmp_path))
    assert r['weld'] is None
    assert r['ring_z'] == (1.0, 1.0) and r['tilt_peak'] == 4.0
    assert [p for _, p in r['approach']] == ['climb', 'transit']
    assert abs(r['approach'][0][0] - 15.1) < 0.06 and abs(r['approach'][1][0] - 21.0) < 0.06
    # hover at (0, 0.95, 1.6) vs the carrier at 30 deg (0.606, 0.35, 1.5)
    want = np.linalg.norm(np.array([0, 0.95, 1.6]) - np.array([0.7 * np.cos(np.pi / 6), 0.35, 1.5]))
    assert abs(r['separation'][0] - round(want, 3)) < 1e-3
    assert r['disarm_z'] == 1.5
    assert r['at_landed'] == {'z': 1.5, 'armed': False, 'down_before': False}
    assert r['step_max_r'] == 1.3 and r['step_z'] == (1.6, 1.6)
