#!/usr/bin/env python3
"""tools/n2_bands.py -- the N2 twin bands for the rig's carry cells (card docs/experiments/2026-10-16_bands_1m.md).

    python3 tools/n2_bands.py                       # the band table from the twin runs (registry, voids dropped)
    python3 tools/n2_bands.py --score <run_dir> <cell>   # one rig flight (tools/rig_to_run.py output) against its band

Rule (Wesley 17 Oct): band = [max(0, min_twin - 2 sigma_rig), max_twin + 2 sigma_rig], f = 1, per metric:
- path RMSE: the ring's 3-D distance to its published reference, from the path's first moving sample
  (metrics.sweep_window) to LAND;
- peak ring tilt over the same window (the lift is outside it: the rig's lift bob is not a carry number).
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


def path_metrics(run_dir):
    """(path RMSE in m, peak ring tilt in deg) from the path's start to LAND."""
    df = pd.read_csv(os.path.join(run_dir, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = list(csv.DictReader(open(os.path.join(run_dir, 'logs', 'events.csv'))))
    t_land = [float(e['sim_time']) for e in ev if e['event'] == 'LAND']
    t = df.t.to_numpy()
    sweep = M.sweep_window(t, df[['payload_ref_x', 'payload_ref_y']].to_numpy())
    if not sweep.any():
        raise ValueError(f'{run_dir}: the reference never moves')
    w = (t >= t[sweep][0]) & (t <= (t_land[0] if t_land else t[sweep][-1]))
    e = (df[['payload_x', 'payload_y', 'payload_z']].to_numpy()
         - df[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy())[w]
    e = e[np.all(np.isfinite(e), axis=1)]
    return float(np.sqrt(np.mean(np.sum(e ** 2, axis=1)))), float(np.nanmax(df.payload_tilt_deg.to_numpy()[w]))


def sigma_rig():
    df = pd.read_csv(os.path.join(RIG_HOVER, 'logs', 'run.csv')).drop_duplicates('t')
    t_land = [float(e['sim_time']) for e in csv.DictReader(open(os.path.join(RIG_HOVER, 'logs', 'events.csv')))
              if e['event'] == 'LAND'][0]
    w = df[(df.t >= t_land - 8.0) & (df.t < t_land)]
    e = (w[['payload_x', 'payload_y', 'payload_z']].to_numpy()
         - w[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy())
    return float(np.linalg.norm(e, axis=1).std(ddof=1)), float(w.payload_tilt_deg.std(ddof=1))


def twin_runs():
    rows = list(csv.DictReader(open(os.path.join(REPO, 'results', 'registry.csv'))))
    void = {r['run'] for r in rows if r['verdict'].startswith('VOID')}
    void |= {r['run'][:-1] for r in rows if re.fullmatch(r'R\d{4}c', r['run']) and 'VOID' in r['verdict'] + r['notes']}
    out = {}
    for r in rows:
        cfg = r['config'].split()[0] if r['config'] else ''
        # the 16 Oct tree onwards (R1877, the carry claim's 1 m arm); older 1 m runs are another tree
        if re.fullmatch(r'R\d{4}', r['run']) and cfg.startswith('traj3_L100_') and int(r['run'][1:]) >= 1877 \
                and r['run'] not in void:
            out.setdefault(cfg[len('traj3_L100_'):], []).append(r['run'])
    return out


def bands():
    s_r, s_t = sigma_rig()
    runs, table = twin_runs(), {}
    for c in CELLS:
        vals = [path_metrics(glob.glob(os.path.join(REPO, 'results', '*', f'{r}_*'))[0]) for r in runs.get(c, [])]
        rm, tl = [v[0] for v in vals], [v[1] for v in vals]
        table[c] = dict(runs=runs.get(c, []), rmse=rm, tilt=tl,
                        rmse_band=[max(0.0, min(rm) - 2 * s_r), max(rm) + 2 * s_r],
                        tilt_band=[max(0.0, min(tl) - 2 * s_t), max(tl) + 2 * s_t])
    return s_r, s_t, table


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--score', nargs=2, metavar=('RUN_DIR', 'CELL'))
    ap.add_argument('--json', help='write the bands here')
    a = ap.parse_args(argv)
    s_r, s_t, table = bands()
    if a.score:
        c = table[a.score[1]]
        rm, tl = path_metrics(a.score[0])
        for name, v, b, k in (('path RMSE (mm)', 1000 * rm, c['rmse_band'], 1000), ('ring tilt (deg)', tl, c['tilt_band'], 1)):
            mid = k * (min(c['rmse' if k == 1000 else 'tilt']) + max(c['rmse' if k == 1000 else 'tilt'])) / 2
            hit = k * b[0] <= v <= k * b[1]
            print(f'{name}: rig {v:.2f}, band {k * b[0]:.2f}-{k * b[1]:.2f} -> {"HIT" if hit else "MISS"}; '
                  f'log(rig / twin midpoint) {np.log(v / mid):+.2f}')
        return
    print(f'sigma_rig: path error {1000 * s_r:.1f} mm, ring tilt {s_t:.2f} deg (RIG-1014-lift3_3, last 8 s of hover)\n')
    print('| cell | twin runs | path RMSE (mm) | band (mm) | ring tilt peak (deg) | band (deg) |')
    print('|---|---|---|---|---|---|')
    for c in CELLS:
        r = table[c]
        print(f"| {c} | {', '.join(r['runs'])} | {' / '.join(f'{1000 * x:.1f}' for x in r['rmse'])} | "
              f"{1000 * r['rmse_band'][0]:.1f}-{1000 * r['rmse_band'][1]:.1f} | "
              f"{' / '.join(f'{x:.2f}' for x in r['tilt'])} | {r['tilt_band'][0]:.2f}-{r['tilt_band'][1]:.2f} |")
    if a.json:
        json.dump(dict(sigma_rig_m=s_r, sigma_rig_deg=s_t, cells=table), open(a.json, 'w'), indent=1)


if __name__ == '__main__':
    main(sys.argv[1:])
