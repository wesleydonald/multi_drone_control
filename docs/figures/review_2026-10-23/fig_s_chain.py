#!/usr/bin/env python3
"""S: the 8 -> 4 -> 8 chain (card docs/experiments/2026-10-23_scalability.md, N; exploratory run R2597).

Message: eight quads carry the 0.9 kg, 0.5 m ring through four unannounced releases down to four and four rejoins
back to eight, with the ring's tilt never above about 3.4 deg.
Hero: the ring's tilt (run.csv, as recorded) from the lift's top to LAND; above it, the planner's fleet size (its log,
moved onto the run clock by the manifest's t_ready_sim, as tools/s_metrics.py does). Events from events.csv. Left out:
the lift, the landing, the rejoiners' approaches (fig_rejoin), the bars (R2597 is exploratory).
Preliminary: one exploratory run at 1 SQP iteration; the claim runs fly at 5 (Q6) and are owed.
"""
import csv
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402

RUN = 'R2597'


def main():
    d = glob.glob(os.path.join(S.REPO, 'results', '*', f'{RUN}_*'))[0]
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    ev = [(float(e['sim_time']), e['event'], e['arg']) for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv')))]
    t_lift = next(t for t, n, _ in ev if n == 'LIFTED')
    t_land = next(t for t, n, _ in ev if n == 'LAND')
    t_ready = float(json.load(open(os.path.join(d, 'manifest.json')))['t_ready_sim'])
    log = pd.read_csv(glob.glob(os.path.join(d, 'logs', 'dissipative_planner', '*', 'log.csv'))[0],
                      usecols=['sim_time', 'n'])
    tn, n = log.sim_time.to_numpy() - t_ready, log.n.to_numpy()
    t = run.t.to_numpy()
    w = (t >= t_lift) & (t <= t_land)
    wn = (tn >= t_lift) & (tn <= t_land)

    S.setup()
    fig = S.figure('full', 2.6)
    gs = fig.add_gridspec(2, 1, height_ratios=[1, 2.6])
    ax_n = fig.add_subplot(gs[0])
    ax = fig.add_subplot(gs[1], sharex=ax_n)
    # the fleet-size palette (plasma: 4 at 0.20 as in the best set, 8 at 0)
    colour = {k: S.plt.cm.plasma(0.05 * (8 - k)) for k in range(4, 9)}
    ts, ns = tn[wn] - t_lift, n[wn]
    cuts = np.r_[0, np.flatnonzero(np.diff(ns)) + 1, len(ns)]
    for a, b in zip(cuts[:-1], cuts[1:]):
        x1 = ts[b] if b < len(ts) else ts[-1]
        xs, ys = ([ts[a]] * 2, [ns[a - 1], ns[a]]) if a else ([ts[a]], [ns[a]])
        ax_n.plot(xs + [x1], ys + [ns[a]], color=colour[int(ns[a])], lw=S.LW_HERO, solid_capstyle='butt')
    ax_n.set_ylim(3.5, 8.5)
    ax_n.set_yticks([4, 6, 8])
    ax_n.set_ylabel('Quads')
    ax_n.tick_params(labelbottom=False)
    for tt, name, arg in ev:
        if name in ('RELEASE', 'REJOIN_WELD') and t_lift <= tt <= t_land:
            for a in (ax_n, ax):
                a.axvline(tt - t_lift, color=S.LIGHT, lw=0.5, zorder=0)
    rel = [tt - t_lift for tt, name, _ in ev if name == 'RELEASE']
    weld = [tt - t_lift for tt, name, _ in ev if name == 'REJOIN_WELD']
    ax_n.annotate('releases', xy=(np.mean([rel[0], rel[-1]]), 1.0), xycoords=('data', 'axes fraction'),
                  xytext=(0, 3), textcoords='offset points', ha='center', va='bottom', fontsize=8, color=S.GREY)
    ax_n.annotate('rejoins', xy=(np.mean([weld[0], weld[-1]]), 1.0), xycoords=('data', 'axes fraction'),
                  xytext=(0, 3), textcoords='offset points', ha='center', va='bottom', fontsize=8, color=S.GREY)
    tilt = run.payload_tilt_deg.to_numpy()
    ax.plot(t[w] - t_lift, tilt[w], color='#0d0887', lw=S.LW_CTX, rasterized=True)
    print(f'tilt peak {np.nanmax(tilt[w]):.2f} deg at {t[w][np.nanargmax(tilt[w])] - t_lift:.1f} s after the lift; '
          f'n {sorted(set(n[wn].tolist()))}; window {t_land - t_lift:.1f} s')
    ax.set_ylim(0, 4)
    ax.set_yticks([0, 1, 2, 3, 4])
    ax.set_ylabel(r'Ring tilt ($^\circ$)')
    ax.set_xlim(0, t_land - t_lift)
    ax.set_xlabel(r'$t$ after the lift (s)')
    fig.align_ylabels([ax_n, ax])
    S.ps.panel_label(ax_n, 'a')
    S.ps.panel_label(ax, 'b')
    S.save(fig, 'fig_s_chain')


if __name__ == '__main__':
    main()
