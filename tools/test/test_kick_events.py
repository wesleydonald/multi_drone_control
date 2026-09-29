"""kick_events finds a glitch kick, a stall kick and a scheduled fleet-wide step in a synthetic run."""
import csv
import sys
import pathlib

import numpy as np
import pandas as pd

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import kick_events  # noqa: E402

COLS = ['step', 'sim_time', 'u0', 'u1', 'u2', 'u3', 'pose_x', 'pose_y', 'pose_z', 'pose_qw', 'pose_qx',
        'pose_qy', 'pose_qz', 'ref_x', 'ref_y', 'ref_z', 'wx', 'wy', 'wz', 'qref_w', 'qref_x', 'qref_y',
        'qref_z', 'solve_status']


def _log(path, drone):
    t = np.round(np.arange(0.0, 40.0, 0.02), 3)
    n = len(t)
    d = {c: np.zeros(n) for c in COLS}
    d['step'] = np.arange(n)
    d['sim_time'] = t
    d['u2'] = np.where(t >= 1.0, 0.5, 0.0)
    d['pose_x'] = 0.001 * t                       # slow drift: no stale ticks
    d['pose_z'] = np.full(n, 1.0)
    d['pose_qw'] = np.ones(n)
    d['ref_z'] = np.where(t >= 30.0, 1.05, 1.0)   # fleet-wide reference step ...
    d['qref_w'] = np.ones(n)
    d['u2'] = d['u2'] + np.where(t >= 30.0, 0.03, 0.0)   # ... and its throttle step on every drone
    keep = np.ones(n, bool)
    if drone == 0:
        k = np.searchsorted(t, 20.0)
        d['wx'][k - 1] = 1.0                      # x0 rate glitch: attitude does not move
        d['u2'] = d['u2'] + np.where(t >= 20.0, 0.03, 0.0) + np.where(t >= 20.1, 0.03, 0.0)  # merged
    else:
        keep[np.searchsorted(t, 25.0)] = False    # one missing tick -> 40 ms stall
        d['u2'] = d['u2'] + np.where(t >= 25.04, 0.03, 0.0)
    path.mkdir(parents=True)
    pd.DataFrame({c: d[c][keep] for c in COLS}).to_csv(path / 'log.csv', index=False)


def _run(tmp_path):
    run = tmp_path / 'R9999_sim_gz_kicks'
    (run / 'logs').mkdir(parents=True)
    with open(run / 'logs' / 'events.csv', 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerows([['sim_time', 'event', 'arg'], [2.0, 'TAKEOFF', ''], [38.0, 'LAND', '']])
    for dr in (0, 1):
        _log(run / 'logs' / 'controller_quad_load' / f'planner_drone{dr}_20260928_000000', dr)
    return run


def test_events_classified(tmp_path):
    T, _ = kick_events.run_ticks(str(_run(tmp_path)))
    ev = kick_events.events_table('R9999', T).set_index(['drone', 't'])
    assert len(ev) == 4
    glitch = ev.loc[(0, 20.0)]
    assert glitch.scheduled == 0 and glitch.rate_jump == 1 and glitch.rate_inconsistent == 1
    assert glitch.stall5 == 0
    stall = ev.loc[(1, 25.04)]
    assert stall.scheduled == 0 and stall.stall5 == 1 and stall.rate_jump == 0
    assert ev.loc[(0, 30.0)].scheduled == 1 and ev.loc[(1, 30.0)].scheduled == 1


def test_scored_per_drone_second(tmp_path):
    T, _ = kick_events.run_ticks(str(_run(tmp_path)))
    r = kick_events.summary_row('R9999', T)
    assert (r['events'], r['sched'], r['unsched']) == (4, 2, 2)
    # window TAKEOFF+3 s (5.0) .. LAND (38.0) on two drones, minus the edge ticks
    assert abs(r['air_drone_s'] - 66.0) < 0.2
    assert abs(r['per100ds'] - 100 * 2 / r['air_drone_s']) < 0.01


def test_cli_writes_csv(tmp_path):
    out = tmp_path / 'ev.csv'
    assert kick_events.main([str(_run(tmp_path)), '-o', str(out)]) == 0
    assert len(pd.read_csv(out)) == 4


def test_tracker_logs_with_the_x0_columns_parse_the_same(tmp_path, monkeypatch):
    """The tracker appends kt_hat, x0_v* and x0_w* (+ fallback) as its last columns."""
    base = kick_events.events_table('R9999', kick_events.run_ticks(str(_run(tmp_path / 'a')))[0])
    monkeypatch.setitem(globals(), 'COLS', COLS + ['kt_hat', 'x0_vx', 'x0_vy', 'x0_vz', 'x0_wx',
                                                   'x0_wy', 'x0_wz', 'x0_w_fallback'])
    ext = kick_events.events_table('R9999', kick_events.run_ticks(str(_run(tmp_path / 'b')))[0])
    pd.testing.assert_frame_equal(base.reset_index(drop=True), ext.reset_index(drop=True))
