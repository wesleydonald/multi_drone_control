#!/usr/bin/env python3
"""tools/lag_metrics.py -- the planner-clock lag accounting of the 8 Oct lag study, per run.

    python3 tools/lag_metrics.py R1152 R1153 results/rig/2026-10-07/r206e60_logs

On the planner's own clock (its log.csv: x0 = load_x/y, traj_t, plan5_x/y), over the constant-speed
part of a circle or figure-8 (from ORBIT_RAMP_S + 1 s after the path starts to LAND):
  L0  how far behind its reference the measured ring is, in seconds of path: traj_t minus the
      path time whose point is nearest x0
  L5  the same for the plan's node 5 against the reference 0.5 s ahead
  c   the fraction of the gap the plan recovers by node 5: (L0 - L5) / L0
  s   the shortfall the fleet then adds: the L0 measured 0.5 s later minus this tick's L5
  D   per drone (tracker log.csv): the time its pose leads its logged node-0 reference (the
      shift that best matches pose(t - D) to ref(t)), the plan's latency as the tracker flies it
  A0  the ring's lag on ONE clock for every arm (time-cascade critic, must-fix 2): the node clock
      minus t_start (the first planner tick with traj_t > 0) minus the path time nearest the ring
      (tracker 0's payload pose). Independent of how traj_t is counted, so a change of that
      convention cannot move it; L0 - A0 is the reference lead the OCP sees (0.1 s today).
The reference is rebuilt from the trajectory parameters in the planner's params.json and the
captured hover point (the ring's first logged position).
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
sys.path.insert(0, os.path.join(REPO, 'src', 'mpc_planner'))
import run_logs  # noqa: E402
from mpc_planner.load_trajectory import ORBIT_RAMP_S, LoadTrajectory  # noqa: E402


def run_path(arg):
    if os.path.isdir(arg):
        return arg
    hits = glob.glob(os.path.join(REPO, 'results', '*', f'{arg}_*'))
    if not hits:
        raise SystemExit(f'no run {arg}')
    return hits[0]


def _nearest_time(traj, hover, xy, t_guess, span=1.5, step=0.002):
    """Path time near t_guess whose reference point is closest to xy."""
    ts = np.arange(max(0.0, t_guess - span), t_guess + span, step)
    pts = np.array([traj.offset_at(t)[:2] for t in ts]) + hover
    return float(ts[np.argmin(np.linalg.norm(pts - xy, axis=1))])


def planner_lags(path):
    csv = sorted(run_logs.planner_csvs(path), key=lambda p: -os.path.getsize(p))[0]
    L = pd.read_csv(csv)
    prm = json.load(open(os.path.join(os.path.dirname(csv), 'params.json')))
    traj = LoadTrajectory(prm.get('load_traj', 'orbit'), float(prm.get('traj_speed', 0.125)),
                          float(prm.get('traj_distance', 1.0)), float(prm.get('traj_radius', 0.5)))
    hover = L[['load_x', 'load_y']].dropna().iloc[0].to_numpy(float)
    run = L[(L.traj_t >= ORBIT_RAMP_S + 1.0) & (L.land.astype(str).isin(['0', 'False', '0.0']))]
    if traj.kind == 'fig_8':
        end = traj._fig8_theta(0.0)[3]
        run = run[run.traj_t <= end - ORBIT_RAMP_S - 1.0]
    run = run.iloc[::2]                                  # 5 Hz is plenty, halves the search
    L0, L5 = [], []
    for _, r in run.iterrows():
        tt = float(r.traj_t)
        L0.append(tt - _nearest_time(traj, hover, np.array([r.load_x, r.load_y]), tt))
        L5.append(tt + 0.5 - _nearest_time(traj, hover, np.array([r.plan5_x, r.plan5_y]), tt + 0.5))
    L0, L5 = np.array(L0), np.array(L5)
    st = run.sim_time.to_numpy()
    later = np.interp(st + 0.5, st, L0, right=np.nan)
    s = later - L5
    t_start = float(L.sim_time[L.traj_t > 0].iloc[0])
    return {'kind': traj.kind, 'speed': traj.speed, 'ticks': len(run), 'traj': traj, 'hover': hover,
            't_start': t_start,
            'L0': float(np.nanmean(L0)), 'L5': float(np.nanmean(L5)),
            'c': float(np.nanmean(L0 - L5) / np.nanmean(L0)), 's': float(np.nanmean(s)),
            'window': (float(run.sim_time.iloc[0]), float(run.sim_time.iloc[-1])) if len(run) else None}


def absolute_lag(path, pl):
    """A0: mean of (t - t_start) - path time nearest the ring, over the planner window."""
    f = sorted(run_logs.node_csvs(path, 'tracker'), key=lambda f: run_logs.drone_of(f) or 0)[0]
    T = pd.read_csv(f).drop_duplicates('sim_time').dropna(subset=['payload_x'])
    T = T[(T.sim_time >= pl['window'][0]) & (T.sim_time <= pl['window'][1])].iloc[::10]
    lags = [(t - pl['t_start']) - _nearest_time(pl['traj'], pl['hover'], np.array([x, y]), t - pl['t_start'])
            for t, x, y in zip(T.sim_time, T.payload_x, T.payload_y)]
    return float(np.mean(lags)) if lags else float('nan')


def tracker_delay(path, window):
    out = {}
    for f in run_logs.node_csvs(path, 'tracker'):
        i = run_logs.drone_of(f)
        T = pd.read_csv(f).drop_duplicates('sim_time')
        T = T[(T.sim_time >= window[0]) & (T.sim_time <= window[1])].dropna(subset=['ref_x', 'pose_x'])
        if len(T) < 50:
            continue
        t = T.sim_time.to_numpy()
        p = T[['pose_x', 'pose_y']].to_numpy()
        ref = T[['ref_x', 'ref_y']].to_numpy()
        lags = np.arange(-0.1, 0.4, 0.005)
        err = [np.nanmean(np.linalg.norm(
            np.c_[np.interp(t - D, t, p[:, 0], left=np.nan), np.interp(t - D, t, p[:, 1], left=np.nan)]
            - ref, axis=1)) for D in lags]
        out[i] = float(lags[int(np.nanargmin(err))])
    return out


def main(args):
    rows = []
    for a in args:
        p = run_path(a)
        pl = planner_lags(p)
        D = tracker_delay(p, pl['window']) if pl['window'] else {}
        A0 = absolute_lag(p, pl) if pl['window'] else float('nan')
        rows.append({'run': os.path.basename(p.rstrip('/'))[:40], 'kind': pl['kind'],
                     'speed': pl['speed'], 'ticks': pl['ticks'],
                     'D (s)': round(float(np.mean(list(D.values()))), 3) if D else None,
                     'D per drone': {k: round(v, 3) for k, v in sorted(D.items())},
                     'A0 (s)': round(A0, 3), 'L0 (s)': round(pl['L0'], 3), 'L5 (s)': round(pl['L5'], 3),
                     'c': round(pl['c'], 3), 's (s)': round(pl['s'], 3)})
    keys = list(rows[0]) if rows else []
    print('| ' + ' | '.join(keys) + ' |')
    print('|' + '---|' * len(keys))
    for r in rows:
        print('| ' + ' | '.join(str(r[k]) for k in keys) + ' |')


if __name__ == '__main__':
    main(sys.argv[1:])
