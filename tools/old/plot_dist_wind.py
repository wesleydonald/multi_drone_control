#!/usr/bin/env python3
"""tools/plot_dist_wind.py -- the option-A wind test (card 2026-10-03_offset_free_mpc) in one figure:
ring drift with the estimate logged only vs applied, and the estimated forces against the wind applied.

    python3 tools/plot_dist_wind.py R0862 R0863 --out results/2026-10-03/b4_wind_n3.png
"""
import argparse
import csv
import glob
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # this file is in tools/old/


def load(rid):
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    ev = list(csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))))
    wr = [(float(e['sim_time']), e['arg']) for e in ev if e['event'] == 'WRENCH']
    on = min(t for t, a in wr if not a.startswith('0 0 0'))
    off = min(t for t, a in wr if a.startswith('0 0 0'))
    truth = {a.split()[3]: float(a.split()[0]) for t, a in wr if not a.startswith('0 0 0')}
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'))
    est_path = glob.glob(os.path.join(d, 'logs', '*planner', '*', 'dist_est.csv'))[0]
    est = pd.read_csv(est_path)
    # the planner logs absolute sim time; the runner's clock starts at "stack ready": align them
    # on the moment the ring first passes 0.3 m in both logs
    tick = pd.read_csv(os.path.join(os.path.dirname(est_path), 'log.csv'))
    up_run = float(run.t[run.payload_z > 0.3].iloc[0])
    up_plan = float(tick.sim_time[tick.load_z > 0.3].iloc[0])
    est['t'] = est['t'] - (up_plan - up_run)
    mode = 'applied' if (est[['app_dL_x', 'app_dD_x']].abs().max().max() > 0) else 'logged only'
    return dict(rid=rid, on=on, off=off, truth=truth, run=run, est=est, mode=mode)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    runs = [load(r) for r in a.runs]
    fig, ax = plt.subplots(3, 1, figsize=(8, 8.5), sharex=True)
    colors = {'logged only': '#c0392b', 'applied': '#1f6fb2'}
    for r in runs:
        t0, run, est = r['on'], r['run'], r['est']
        pre = run[(run.t >= t0 - 5) & (run.t < t0)]
        x0, y0 = pre.payload_x.mean(), pre.payload_y.mean()
        tt = run.t - t0
        keep = (tt > -8) & (tt < r['off'] - t0 + 12)
        drift = 100 * np.hypot(run.payload_x - x0, run.payload_y - y0)
        c = colors[r['mode']]
        lab = f"{r['rid']}: estimate {r['mode']}"
        ax[0].plot(tt[keep], drift[keep], color=c, label=lab)
        te = est.t - t0
        ke = (te > -8) & (te < r['off'] - t0 + 12)
        ax[1].plot(te[ke], est.dL_x[ke], color=c, label=lab)
        ax[2].plot(te[ke], est.dD_x[ke], color=c, label=lab)
    r = runs[0]
    w = r['off'] - r['on']
    ring_true = r['truth'].get('lift_system::payload::body', 0.0)
    drone_true = np.mean([v for k, v in r['truth'].items() if 'x3_drone' in k] or [0.0])
    for i, true in ((1, ring_true), (2, drone_true)):
        ax[i].step([-8, 0, w, w + 12], [0, true, 0, 0], where='post', color='k', ls='--', lw=1.2,
                   label='wind applied (truth)')
    for a_ in ax:
        a_.axvspan(0, w, color='#f2d7a0', alpha=0.35, lw=0)
        a_.grid(alpha=0.3)
    ax[0].set_ylabel('ring xy drift from hold [cm]')
    ax[1].set_ylabel('ring force estimate, x [N]')
    ax[2].set_ylabel('per-drone force estimate, x [N]')
    ax[2].set_xlabel('time from wind on [s]')
    ax[0].set_title('Wind along -x: 0.5 N on the ring, 0.15 N on each drone (rig twin, 3 drones, 1.0 m)', fontsize=10)
    for a_ in ax:
        a_.legend(fontsize=8, loc='best')
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)
    print(a.out)


if __name__ == '__main__':
    main()
