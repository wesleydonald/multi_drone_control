#!/usr/bin/env python3
"""tools/corner_metrics.py -- corner cutting and lag per run, on the ruler Wesley chose (16 Oct).

    python3 tools/corner_metrics.py R1982 R1986 ...
    python3 tools/corner_metrics.py R1847-R1991           # every trajectory run in the range
    python3 tools/corner_metrics.py --chain --D R1982     # + where along the chain, + plan age

Read-only: it never writes metrics.json. Every number comes from metrics.contour_lag (the ring
against the path it was sent, clock-free, and its lag in seconds on the planner's schedule) over
the fast part of the motion, outside clock holds. Columns (mm unless stated):
  cut / max      signed distance inside the curve over the path's tightest quarter, mean / max
  across         signed distance to the path (+ = inside), mean / rms
  lag            seconds behind the schedule (+ = late), mean / sd
  sched          the ring against the continuous schedule (cut and lag in one number)
  frozen         metrics.json payload_rmse_sweep_m (the 10 Hz held desired; for continuity only)
  lap R / c      on a constant-curvature path: per-lap fitted radius shortfall (+ = inside) mean,
                 its sd over the laps, and the fitted centre's offset from the path's
  plan5          (--chain) the planner's own planned ring 0.5 s ahead (log.csv plan5_x/y): its cut;
                 dref / dact the drones' reference and actual centroids (inside the ring by design)
  tilt           ring tilt max (deg); drone tilt max (deg); throttle max
  solve          planner solve p50 / p95 (ms) over UNIQUE path ticks (log.csv repeats its last
                 solve_ms after the path ends)
  D              (--D) the plan's age as flown, mean over drones (lag_metrics.tracker_delay)
"""
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import metrics as M  # noqa: E402
import run_logs  # noqa: E402

FAULTS = ('FLEET ABORT', 'DROP AND LAND', 'UNANNOUNCED DETACH', 'ENVELOPE FAULT')


def run_dirs(args):
    out = []
    for a in args:
        m = re.fullmatch(r'R(\d{4})-R(\d{4})', a)
        if m:
            for i in range(int(m.group(1)), int(m.group(2)) + 1):
                out += glob.glob(os.path.join(REPO, 'results', '*', f'R{i:04d}_*'))
        elif os.path.isdir(a):
            out.append(a)
        else:
            hits = glob.glob(os.path.join(REPO, 'results', '*', f'{a}_*'))
            if not hits:
                raise SystemExit(f'no run {a}')
            out.append(hits[0])
    return [d for d in out if os.path.exists(os.path.join(d, 'logs', 'run.csv'))]


def _lift(t, z):
    t, z = np.asarray(t, float), np.asarray(z, float)
    up = np.where(z > np.nanmedian(z[:20]) + 0.03)[0]
    return float(t[up[0]]) if up.size else None


def planner_log(d, run):
    """The planner's log.csv with a run_t column on run.csv's clock (aligned on lift-off)."""
    csvs = sorted(run_logs.planner_csvs(d), key=lambda p: -os.path.getsize(p))
    if not csvs:
        return None
    L = pd.read_csv(csvs[0])
    a, b = _lift(L.sim_time, L.load_z), _lift(run.t, run.payload_z)
    if a is None or b is None:
        return None
    L['run_t'] = L.sim_time - (a - b)
    return L


def per_lap(o, xy):
    """Per lap of a constant-curvature path: fitted radius shortfall (+ = inside) and centre
    offset, ring against the path over the same arc. None on a path whose curvature varies."""
    Q, s, _, _, kappa = o['path'][:5]
    use = o['use']
    k = np.abs(o['kappa_r'][use])
    if k.size < 50 or np.std(k) > 0.15 * np.mean(k):
        return None
    lap = 2 * np.pi / np.mean(k)
    sr = o['s_r']
    s0, s1 = np.nanmin(sr[use]), np.nanmax(sr[use])
    n = max(1, int((s1 - s0) // lap))
    dR, dc = [], []
    for i in range(n):
        a, b = s0 + i * lap, s0 + (i + 1) * lap if n > 1 else s1
        r = use & (sr >= a) & (sr < b)
        p = (s >= a) & (s < b)
        if np.count_nonzero(r) < 30:
            continue
        cr, Rr = M.fit_circle(xy[r])
        cp, Rp = M.fit_circle(Q[p])
        dR.append(Rp - Rr)
        dc.append(float(np.linalg.norm(cr - cp)))
    return (1000 * np.mean(dR), 1000 * np.std(dR), 1000 * np.mean(dc), len(dR)) if dR else None


def score(d, chain=False, D=False):
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    t = run.t.to_numpy()
    des = run[['payload_ref_x', 'payload_ref_y']].to_numpy()
    xy = run[['payload_x', 'payload_y']].to_numpy()
    sweep = M.sweep_window(t, des)
    o = M.contour_lag(t, xy, des, sweep, full=True)
    if not o:
        return None
    use = o['use']
    name = os.path.basename(d.rstrip('/'))
    rid, cfg = name.split('_')[0], name.split('_sim_gz_')[-1]
    r = {'run': rid, 'cfg': cfg}
    mm = lambda v: 1000.0 * v  # noqa: E731
    r.update(cut=mm(o['cut_topq_mean_m']), cut_max=mm(o['cut_topq_max_m']),
             across=mm(o['across_mean_m']), across_rms=mm(o['across_rms_m']),
             lag=o['lag_s'], lag_sd=o['lag_sd_s'], sched=mm(o['rmse_sched_m']))
    mf = os.path.join(d, 'metrics.json')
    m = json.load(open(mf)) if os.path.exists(mf) else {}
    r['frozen'] = mm(m.get('payload_rmse_sweep_m', np.nan))
    lap = per_lap(o, xy)
    r['lap_R'], r['lap_R_sd'], r['lap_c'], r['laps'] = lap if lap else (np.nan, np.nan, np.nan, 0)
    w = run[use]
    n = len([c for c in run.columns if re.fullmatch(r'd\d+_ref_x', c)])
    r['tilt'] = float(w.payload_tilt_deg.max())
    r['drone_tilt'] = float(max(w[f'd{i}_tilt_deg'].max() for i in range(n)))
    r['thr'] = float(max(w[f'd{i}_thr'].max() for i in range(n)))
    L = planner_log(d, run)
    if L is not None:
        t0, t1 = float(t[use][0]), float(t[use][-1])
        P = L[(L.run_t >= t0) & (L.run_t <= t1)]
        P = P[P.traj_t.diff().fillna(1.0) > 1e-6]            # unique ticks only
        r['solve50'] = float(np.nanpercentile(P.solve_ms, 50)) if len(P) else np.nan
        r['solve95'] = float(np.nanpercentile(P.solve_ms, 95)) if len(P) else np.nan
        r['fails'] = int((P.solve_status != 0).sum()) if 'solve_status' in P else 0
        if chain and 'plan5_x' in P:
            tk, s_k = o['knots'][0], o['path'][5]
            seed = np.interp(P.run_t.to_numpy() + 0.5, tk, s_k)
            c5, _, k5 = M.project_on_path(o['path'], P[['plan5_x', 'plan5_y']].to_numpy(), seed)
            top = np.abs(k5) >= o['topq_curvature']
            r['plan5'] = mm(np.nanmean(c5[top])) if np.any(top) else np.nan
        if D:
            import lag_metrics
            Dd = lag_metrics.tracker_delay(d, (float(P.sim_time.min()), float(P.sim_time.max())))
            r['D'] = float(np.mean(list(Dd.values()))) if Dd else np.nan
    if chain:
        top = o['top']
        for key, fmt in (('dref', 'd{i}_ref_{a}'), ('dact', 'd{i}_{a}')):
            c = np.column_stack([np.mean([run[fmt.format(i=i, a=a)] for i in range(n)], axis=0) for a in 'xy'])
            cc, _, _ = M.project_on_path(o['path'], c, np.where(use, o['s_sched'], np.nan))
            r[key] = mm(np.nanmean(cc[top])) if np.any(top) else np.nan
    log = os.path.join(d, 'logs', 'launch.log')
    text = open(log, errors='replace').read() if os.path.exists(log) else ''
    r['faults'] = ','.join(f for f in FAULTS if f in text) or 'ok'
    return r


COLS = [('run', '{}'), ('cfg', '{}'), ('cut', '{:+.1f}'), ('cut_max', '{:+.1f}'), ('across', '{:+.1f}'),
        ('across_rms', '{:.1f}'), ('lag', '{:+.3f}'), ('lag_sd', '{:.3f}'), ('sched', '{:.1f}'),
        ('frozen', '{:.1f}'), ('lap_R', '{:+.1f}'), ('lap_R_sd', '{:.1f}'), ('lap_c', '{:.1f}'), ('laps', '{}'),
        ('plan5', '{:+.1f}'), ('dref', '{:+.1f}'), ('dact', '{:+.1f}'), ('tilt', '{:.1f}'),
        ('drone_tilt', '{:.1f}'), ('thr', '{:.3f}'), ('solve50', '{:.0f}'), ('solve95', '{:.0f}'),
        ('fails', '{}'), ('D', '{:.3f}'), ('faults', '{}')]


def main(argv):
    chain, D = '--chain' in argv, '--D' in argv
    rows = [r for r in (score(d, chain, D) for d in run_dirs([a for a in argv if not a.startswith('--')])) if r]
    cols = [(k, f) for k, f in COLS if any(k in r for r in rows)]
    print('| ' + ' | '.join(k for k, _ in cols) + ' |')
    print('|' + '---|' * len(cols))
    for r in rows:
        cells = []
        for k, f in cols:
            v = r.get(k, '')
            cells.append('' if v == '' or (isinstance(v, float) and not np.isfinite(v)) else f.format(v))
        print('| ' + ' | '.join(cells) + ' |')


if __name__ == '__main__':
    main(sys.argv[1:])
