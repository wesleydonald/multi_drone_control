"""tools/x4_metrics.py on a synthetic relay run (card 2026-10-30_x4_endurance_relay): the ring flies x4_relay_short's
laps exactly (zero path error) with a 4 mm height sag growing lap by lap, its three leaves are released, parked and
welded back, and a 2 deg tilt step marks the first release; the scorer reads the swaps, the per-lap error and drift,
the event tilt and the pack clocks the rota implies."""
import json
import os
import sys

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import x4_metrics  # noqa: E402
from experiment.config import ExperimentConfig  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CFG = os.path.join(REPO, 'configs', 'experiments', 'x4_relay_short.yaml')
T_TAKEOFF, T_PATH, HOLD = 6.0, 31.0, 20.0


def _fake_run(tmp_path):
    plan = ExperimentConfig.from_yaml(CFG).relay
    T = plan.geo.lap_s
    traj = plan.geo.traj
    ev, t_lap, laps = [(T_TAKEOFF, 'TAKEOFF', '')], [], []
    ev.append((T_PATH, 'PATH_START', 'fig_8'))
    t = T_PATH
    for k in range(1, plan.laps + 1):
        laps.append((t, t + T))
        t += T
        if k < plan.laps:
            ev.append((t, 'LAP_DONE', str(k)))
            t_lap.append(t)
            t += HOLD
    t_land = t + 2.0
    ev += [(t_land, 'LAND', ''), (t_land + 15.0, 'LANDED', '/fleet/landed')]
    for i, lv in enumerate(plan.leaves):
        te = t_lap[lv.lap - 1]
        ev += [(te + 1.0, 'DETACH', str(lv.d)), (te + 5.0, 'DETACH_RESIZE', str(lv.d))]
        for j in range(len(lv.wps)):
            ev.append((t_lap[lv.lap - 1] + HOLD + 3.0 + j, 'TASK_AT', str(lv.d)))
        ev.append((t_lap[lv.lap - 1] + HOLD + 10.0, 'TASK_PARKED', str(lv.d)))
        ev.append((t_lap[lv.lap] + 8.0, 'REJOIN_WELD', str(lv.d)))
    ev.sort()
    ts = np.round(np.arange(0.0, t_land + 16.0, 0.05), 3)
    x, y = np.zeros_like(ts), np.zeros_like(ts)
    lap_of = np.zeros_like(ts)
    for k, (a, b) in enumerate(laps, 1):
        w = (ts >= a) & (ts < b)
        off = np.array([traj.offset_at(s - a + 1e-9)[:2] for s in ts[w]])
        x[w], y[w] = off[:, 0], off[:, 1]
        lap_of[(ts >= a)] = k
    z_ref = np.full_like(ts, 1.0)
    z = z_ref - 0.004 * (lap_of - 1)                                     # 4 mm more sag each lap
    tilt = np.where((ts >= t_lap[0] + 5.0) & (ts < t_lap[0] + 7.0), 2.0, 0.0)
    a = np.radians(tilt) / 2.0
    run = pd.DataFrame({'t': ts, 'payload_x': x, 'payload_y': y, 'payload_z': z, 'payload_qw': np.cos(a),
                        'payload_qx': np.sin(a), 'payload_qy': 0.0, 'payload_qz': 0.0, 'payload_tilt_deg': tilt,
                        'payload_ref_x': x, 'payload_ref_y': y, 'payload_ref_z': z_ref})
    for d in range(8):
        run[f'd{d}_attached'] = 1
    d = tmp_path / 'R9999_sim_gz_x4_relay_short'
    (d / 'logs').mkdir(parents=True)
    run.to_csv(d / 'logs' / 'run.csv', index=False)
    pd.DataFrame(ev, columns=['sim_time', 'event', 'arg']).to_csv(d / 'logs' / 'events.csv', index=False)
    (d / 'manifest.json').write_text(json.dumps({'scenario': CFG}))
    return str(d), plan, t_lap, t_land + 15.0


def test_the_scorer_reads_the_swaps_laps_tilt_and_packs(tmp_path):
    d, plan, t_lap, t_down = _fake_run(tmp_path)
    r = x4_metrics.score(d)
    assert r['mission_s'] == pytest.approx(t_down - T_TAKEOFF) and r['mission_over_b'] == pytest.approx(
        (t_down - T_TAKEOFF) / plan.budget)
    assert (r['swaps_tried'], r['released'], r['parked'], r['welded_back'], r['swaps_done']) == (3, 3, 3, 3, 3)
    assert not r['faults'] and not r['dropped']
    assert len(r['laps']) == plan.laps
    assert max(lp['path_rms_m'] for lp in r['laps'][:1]) < 1e-3                 # the first lap: on the path
    assert r['height_drift_m'] == pytest.approx(-0.004 * (plan.laps - 1), abs=1e-3)
    assert r['slope_mm_per_lap'] == pytest.approx(4.0, abs=0.3) and r['last_over_first'] > 1.0
    first = r['events'][0]
    assert first['kind'] == 'release' and first['peak_deg'] == pytest.approx(2.0, abs=0.05)
    assert r['packs_matched_rota']
    lv0 = plan.leaves[0]
    arrive = t_lap[lv0.lap - 1] + HOLD + 3.0 + len(lv0.wps) - 1
    assert r['packs'][str(lv0.d)][0] == pytest.approx(arrive - T_TAKEOFF)
    errand = next(lv for lv in plan.leaves if lv.kind == 'errand')
    assert len(r['packs'][str(errand.d)]) == 1                                   # an errand does not recharge
    assert r['max_pack_s'] == pytest.approx(t_down - T_TAKEOFF)                 # a drone that never left
