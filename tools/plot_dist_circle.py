#!/usr/bin/env python3
"""tools/plot_dist_circle.py -- the circle in Gazebo wind (card 2026-10-03_offset_free_mpc): the desired
ring path against the flown ones, with the disturbance estimate logged only and applied.

    python3 tools/plot_dist_circle.py R0864 R0865 --out results/2026-10-03/b4_wind_circle.png
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

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
COLORS = {'logged only': '#c0392b', 'applied': '#1f6fb2'}
LAP_S = 27.2      # one lap of r 0.5 at 0.125 m/s (25.1 s) plus the 4 s smootherstep spin-up (about 2 s)


def load(rid):
    d = glob.glob(os.path.join(REPO, 'results', '*', f'{rid}_*'))[0]
    ev = {e['event']: float(e['sim_time']) for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv')))}
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'))
    est_path = glob.glob(os.path.join(d, 'logs', '*planner', '*', 'dist_est.csv'))[0]
    est = pd.read_csv(est_path)
    tick = pd.read_csv(os.path.join(os.path.dirname(est_path), 'log.csv'))
    # the planner logs absolute sim time, the runner from "stack ready": align on the ring passing 0.3 m
    off = float(tick.sim_time[tick.load_z > 0.3].iloc[0]) - float(run.t[run.payload_z > 0.3].iloc[0])
    est['t'] -= off
    start = float(tick.sim_time[tick.traj_t > 0].iloc[0]) - off
    mode = 'applied' if est[['app_dL_x', 'app_dD_x']].abs().max().max() > 0 else 'logged only'
    end = min(ev['LAND'], start + LAP_S)
    lap = run[(run.t >= start) & (run.t < end)]
    err = np.hypot(lap.payload_x - lap.payload_ref_x, lap.payload_y - lap.payload_ref_y)
    return dict(rid=rid, run=run, est=est, start=start, land=end, mode=mode, lap=lap, err=err)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    runs = [load(r) for r in a.runs]
    fig = plt.figure(figsize=(8, 10.5))
    gs = fig.add_gridspec(3, 1, height_ratios=[2.2, 1, 1])
    ax0 = fig.add_subplot(gs[0])
    ax1 = fig.add_subplot(gs[1])
    ax2 = fig.add_subplot(gs[2], sharex=ax1)
    ref = runs[0]['lap']
    ax0.plot(ref.payload_ref_x, ref.payload_ref_y, 'k--', lw=1.5, label='desired ring path')
    for r in runs:
        c = COLORS[r['mode']]
        lab = f"{r['rid']}: estimate {r['mode']} (xy error mean {100 * r['err'].mean():.1f} cm, max {100 * r['err'].max():.1f})"
        ax0.plot(r['lap'].payload_x, r['lap'].payload_y, color=c, lw=1.8, label=lab)
        ax0.plot(r['lap'].payload_x.iloc[0], r['lap'].payload_y.iloc[0], 'o', color=c, ms=5)
        ax1.plot(r['lap'].t - r['start'], 100 * r['err'], color=c, label=f"{r['rid']}: estimate {r['mode']}")
        e = r['est'][(r['est'].t >= r['start'] - 5) & (r['est'].t < r['land'])]
        ax2.plot(e.t - r['start'], e.dL_x, color=c, label=f"{r['rid']} ring force x")
        ax2.plot(e.t - r['start'], e.dD_x, color=c, ls=':', label=f"{r['rid']} per-drone force x")
    cx, cy = ref.payload_ref_x.mean(), ref.payload_ref_y.mean()
    ax0.annotate('', xy=(cx - 0.25, cy), xytext=(cx + 0.05, cy),
                 arrowprops=dict(arrowstyle='->', color='0.35', lw=2))
    ax0.text(cx - 0.1, cy + 0.03, 'wind 0.4 m/s', ha='center', color='0.35', fontsize=9)
    ax0.set_aspect('equal')
    ax0.set_xlabel('x [m]')
    ax0.set_ylabel('y [m]')
    ax0.set_title('Circle r 0.5 m at 0.125 m/s in a steady wind (rig twin, 3 drones, 1.0 m): one lap', fontsize=10)
    ax0.legend(fontsize=8, loc='upper left', bbox_to_anchor=(0.0, -0.12))
    ax1.set_ylabel('ring xy error [cm]')
    ax2.set_ylabel('force estimate [N]')
    ax2.set_xlabel('time from circle start [s]')
    for a_ in (ax0, ax1, ax2):
        a_.grid(alpha=0.3)
    ax1.legend(fontsize=8)
    ax2.legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(a.out, dpi=150)
    print(a.out)
    for r in runs:
        print(f"{r['rid']} {r['mode']}: lap xy error mean {100 * r['err'].mean():.1f} cm, max {100 * r['err'].max():.1f} cm, "
              f"lap {r['land'] - r['start']:.1f} s")


if __name__ == '__main__':
    main()
