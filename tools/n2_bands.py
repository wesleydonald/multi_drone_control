#!/usr/bin/env python3
"""tools/n2_bands.py -- the N2 twin bands for the rig's carry cells (card docs/experiments/2026-10-16_bands_1m.md).

    python3 tools/n2_bands.py                       # the band table from the twin runs (registry, voids dropped)
    python3 tools/n2_bands.py --score <run_dir> <cell>   # one rig flight (tools/rig_to_run.py output) against its band
    python3 tools/n2_bands.py --rod 0.55 [...]            # the 0.55 m rods (frozen 18 Oct): the carry claim + one more each
    python3 tools/n2_bands.py --set b22 [...]             # the 22 Oct baselines: 2/3/4 quads x hover / figure-8 size 1.0 /
                                                          # constant spin x old filter / slope (R2329 on; card
                                                          # 2026-10-22_velocity_filter); the rig scores against its filter's cell;
                                                          # also the figure-8 at size 0.9 (R2405-R2416) and the rig's 0.75 m
                                                          # paths, figure-8 and spinny line (R2423-R2434)

Rule (Wesley 17 Oct): band = [max(0, min_twin - 2 sigma_rig), max_twin + 2 sigma_rig], f = 1, per metric:
- path RMSE: the ring's 3-D distance to its published reference, from the path's first moving sample
  (metrics.sweep_window) to LAND;
- peak ring tilt over the same window (the lift is outside it: the rig's lift bob is not a carry number).
A hover cell has no moving reference: its window runs from 2 s after the reference reaches its top to LAND (set 22 Oct
for the b22 set, before any of its runs was scored).
The b22 set's path cells score CLOCK-FREE (Wesley 22 Oct: lag is fine as long as the path is followed closely): the
ring's distance to the path the reference traces (sideways) and its height error, RMS from the path's first move to
the start of the planner's own descent (or LAND if earlier); the tilt peak over the same window; the lag is not banded
(corner_metrics reports it). The traj3 sets keep the 17 Oct rule above.
sigma_rig: RIG-1014-lift3_3 (lift3 itself was void), steady hover at 1.00 m, the last 8 s before LAND.
"""
import argparse
import csv
import glob
import json
import os
import re
import sys

import numpy as np
import pandas as pd

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
import metrics as M  # noqa: E402

RIG_HOVER = os.path.join(REPO, 'results', 'rig', '2026-10-14', 'lift3_3_run')
CELLS = [f'{s}_v{v}' for s in ('circle', 'fig8', 'linex', 'orbit', 'spin') for v in ('125', '200', '300')]
B22 = [f'b{n}_{c}_L100_{f}' for n in (2, 3, 4)
       for c in ('hover', 'fig8big_h5_v200', 'linespinc_h5_v200', 'fig8s09_h5_v200', 'fig8s075_h5_v200',
                 'linespinc075_h5_v200')
       for f in ('ema', 'slope')]
B22_FIRST = 2329


def path_metrics(run_dir):
    """(path RMSE in m, peak ring tilt in deg) from the path's start to LAND."""
    df = pd.read_csv(os.path.join(run_dir, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = list(csv.DictReader(open(os.path.join(run_dir, 'logs', 'events.csv'))))
    t_land = [float(e['sim_time']) for e in ev if e['event'] == 'LAND']
    t = df.t.to_numpy()
    sweep = M.sweep_window(t, df[['payload_ref_x', 'payload_ref_y']].to_numpy())
    if sweep.any():
        w = (t >= t[sweep][0]) & (t <= (t_land[0] if t_land else t[sweep][-1]))
    else:                                   # a hover: from 2 s after the reference tops out to LAND
        zr = df.payload_ref_z.to_numpy()
        top = t[np.argmax(zr >= np.nanmax(zr) - 1e-3)] + 2.0
        w = (t >= top) & (t <= (t_land[0] if t_land else t[-1]))
    e = (df[['payload_x', 'payload_y', 'payload_z']].to_numpy()
         - df[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy())[w]
    e = e[np.all(np.isfinite(e), axis=1)]
    return float(np.sqrt(np.mean(np.sum(e ** 2, axis=1)))), float(np.nanmax(df.payload_tilt_deg.to_numpy()[w]))


def path_metrics_clock_free(run_dir, t_from=None):
    """(path error in m, peak ring tilt in deg) for the b22 set: clock-free on a path, as path_metrics in a hover.
    t_from: only rows from then on can start the path (a datum shift steps payload_ref earlier: an attach, a hand-over).
    """
    df = pd.read_csv(os.path.join(run_dir, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    t = df.t.to_numpy()
    des = df[['payload_ref_x', 'payload_ref_y']].to_numpy()
    if t_from is not None:
        des = np.where((t >= float(t_from))[:, None], des, np.nan)
    sweep = M.sweep_window(t, des)
    if not sweep.any():
        return path_metrics(run_dir)
    ev = list(csv.DictReader(open(os.path.join(run_dir, 'logs', 'events.csv'))))
    t_land = [float(e['sim_time']) for e in ev if e['event'] == 'LAND']
    t0 = t[sweep][0]
    zr = df.payload_ref_z.to_numpy()
    i0 = int(np.searchsorted(t, t0))
    down = np.where(zr[i0:] < np.nanmax(zr[i0:]) - 0.02)[0]
    t1 = min(t[i0 + down[0]] if len(down) else t[-1], t_land[0] if t_land else t[-1])
    w = (t >= t0) & (t <= t1)
    tk, xk = M.schedule_knots(t, des, sweep)
    path = M.path_through(xk)
    s_sched = np.interp(t, tk, path[5])
    across, _, _ = M.project_on_path(path, df[['payload_x', 'payload_y']].to_numpy(), np.where(w, s_sched, np.nan))
    dz = df.payload_z.to_numpy() - zr
    e = np.sqrt(across ** 2 + dz ** 2)[w]
    e = e[np.isfinite(e)]
    return float(np.sqrt(np.mean(e ** 2))), float(np.nanmax(df.payload_tilt_deg.to_numpy()[w]))


def sigma_rig():
    df = pd.read_csv(os.path.join(RIG_HOVER, 'logs', 'run.csv')).drop_duplicates('t')
    t_land = [float(e['sim_time']) for e in csv.DictReader(open(os.path.join(RIG_HOVER, 'logs', 'events.csv')))
              if e['event'] == 'LAND'][0]
    w = df[(df.t >= t_land - 8.0) & (df.t < t_land)]
    e = (w[['payload_x', 'payload_y', 'payload_z']].to_numpy()
         - w[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy())
    return float(np.linalg.norm(e, axis=1).std(ddof=1)), float(w.payload_tilt_deg.std(ddof=1))


def twin_runs(rod='1', cell_set='traj3'):
    rows = list(csv.DictReader(open(os.path.join(REPO, 'results', 'registry.csv'))))
    void = {r['run'] for r in rows if r['verdict'].startswith('VOID')}
    void |= {r['run'][:-1] for r in rows if re.fullmatch(r'R\d{4}c', r['run']) and 'VOID' in r['verdict'] + r['notes']}
    out = {}
    for r in rows:
        cfg = r['config'].split()[0] if r['config'] else ''
        if not re.fullmatch(r'R\d{4}', r['run']) or r['run'] in void:
            continue
        k = int(r['run'][1:])
        if cell_set == 'b22':
            if cfg in B22 and k >= B22_FIRST:
                out.setdefault(cfg, []).append(r['run'])
            continue
        # 1 m: the 16 Oct tree onwards (R1877, the carry claim's 1 m arm); older 1 m runs are another tree
        if rod == '1' and cfg.startswith('traj3_L100_') and k >= 1877:
            out.setdefault(cfg[len('traj3_L100_'):], []).append(r['run'])
        # 0.55 m: the carry claim's two per cell (R1847-R1876) and the third flown 17-18 Oct (R2201-R2215)
        elif rod == '0.55' and cfg[len('traj3_'):] in CELLS and (1847 <= k <= 1876 or 2201 <= k <= 2215):
            out.setdefault(cfg[len('traj3_'):], []).append(r['run'])
    return out


def bands(rod='1', cell_set='traj3'):
    s_r, s_t = sigma_rig()
    runs, table = twin_runs(rod, cell_set), {}
    metric = path_metrics_clock_free if cell_set == 'b22' else path_metrics
    for c in (B22 if cell_set == 'b22' else CELLS):
        if not runs.get(c):
            continue
        vals = [metric(glob.glob(os.path.join(REPO, 'results', '*', f'{r}_*'))[0]) for r in runs.get(c, [])]
        rm, tl = [v[0] for v in vals], [v[1] for v in vals]
        table[c] = dict(runs=runs.get(c, []), rmse=rm, tilt=tl,
                        rmse_band=[max(0.0, min(rm) - 2 * s_r), max(rm) + 2 * s_r],
                        tilt_band=[max(0.0, min(tl) - 2 * s_t), max(tl) + 2 * s_t])
    return s_r, s_t, table


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--score', nargs=2, metavar=('RUN_DIR', 'CELL'))
    ap.add_argument('--json', help='write the bands here')
    ap.add_argument('--rod', choices=('1', '0.55'), default='1', help='the carriers\' tether length (m)')
    ap.add_argument('--set', default='traj3', choices=('traj3', 'b22'), help='the cells: the 16 Oct carry claim or '
                    'the 22 Oct baselines')
    a = ap.parse_args(argv)
    s_r, s_t, table = bands(a.rod, a.set)
    if a.score:
        c = table[a.score[1]]
        rm, tl = (path_metrics_clock_free if a.set == 'b22' else path_metrics)(a.score[0])
        for name, v, b, k in (('path RMSE (mm)', 1000 * rm, c['rmse_band'], 1000), ('ring tilt (deg)', tl, c['tilt_band'], 1)):
            mid = k * (min(c['rmse' if k == 1000 else 'tilt']) + max(c['rmse' if k == 1000 else 'tilt'])) / 2
            hit = k * b[0] <= v <= k * b[1]
            print(f'{name}: rig {v:.2f}, band {k * b[0]:.2f}-{k * b[1]:.2f} -> {"HIT" if hit else "MISS"}; '
                  f'log(rig / twin midpoint) {np.log(v / mid):+.2f}')
        return
    print(f'sigma_rig: path error {1000 * s_r:.1f} mm, ring tilt {s_t:.2f} deg (RIG-1014-lift3_3, last 8 s of hover)\n')
    print('| cell | twin runs | path RMSE (mm) | band (mm) | ring tilt peak (deg) | band (deg) |')
    print('|---|---|---|---|---|---|')
    for c in table:
        r = table[c]
        print(f"| {c} | {', '.join(r['runs'])} | {' / '.join(f'{1000 * x:.1f}' for x in r['rmse'])} | "
              f"{1000 * r['rmse_band'][0]:.1f}-{1000 * r['rmse_band'][1]:.1f} | "
              f"{' / '.join(f'{x:.2f}' for x in r['tilt'])} | {r['tilt_band'][0]:.2f}-{r['tilt_band'][1]:.2f} |")
    if a.json:
        json.dump(dict(sigma_rig_m=s_r, sigma_rig_deg=s_t, cells=table), open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main(sys.argv[1:])
