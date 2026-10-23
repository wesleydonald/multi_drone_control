#!/usr/bin/env python3
"""C: the ring's yaw under our planner and under our port of Quan et al.'s network (card
docs/experiments/2026-10-23_paper_method_compare.md, Results).

Message: our planner holds the ring's yaw within a fraction of a degree; the network lets it swing, 22-51 deg in hover
and 14 deg on the 0.4 m/s line at its best gains, and as ported the swing grows until the ring capsizes.
Hero: the ring's yaw from the hand-over, our planner blue, the network red (as ported: light red, ended by the fleet
abort). (a) hover, (b) the 0.4 m/s line. Yaw as tools/c_metrics.py computes it (unwrapped, from the quaternion), here
from its value at the hand-over (our planner hands over to nothing: the moment the network would, the reference's top
+ 1.5 s). Each run is drawn to the end of its scored window (60 s, LAND, or the loss).
Runs: hover: network as ported R2446, at the search's best gains R2685, R2686; our planner R2348, R2351 (B's 4-quad
hover baseline on the same world and plant, 22 Oct tree; C's own c_pl_hover cells are owed). Line: our planner R2448,
the network at its best gains R2689, R2690.
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import c_metrics as C  # noqa: E402

HOVER = dict(ours=('R2348', 'R2351'), net=('R2685', 'R2686'), ported=('R2446',))
LINE = dict(ours=('R2448',), net=('R2689', 'R2690'))


def yaw_run(rid):
    d = C.run_dirs([rid])[0]
    s = C.score(d)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    t = run.t.to_numpy()
    w, x, y, z = (run[f'payload_q{c}'].to_numpy() for c in 'wxyz')
    yaw = np.degrees(np.unwrap(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))))
    t0, t1 = s['t_handover'], s['window'][1]
    sel = (t >= t0) & (t <= t1)
    return t[sel] - t0, yaw[sel] - yaw[sel][0], s


def label(ax, text, x, y, col, ha='left'):
    ax.annotate(text, xy=(x, y), ha=ha, va='center', fontsize=8, color=col)


def main():
    S.setup()
    fig = S.figure('full', 2.9)
    axs = fig.subplots(2, 1, sharex=True)
    for ax, runs in ((axs[0], HOVER), (axs[1], LINE)):
        for key, col, lw in (('ported', S.RED_LIGHT, S.LW_CTX), ('net', S.RED, S.LW_CTX), ('ours', S.BLUE, S.LW_HERO)):
            for rid in runs.get(key, ()):
                tt, yy, s = yaw_run(rid)
                ax.plot(tt, yy, color=col, lw=lw, zorder=3 if key == 'ours' else 2)
                print(rid, key, s['case'], f"max |yaw| {np.max(np.abs(yy)):.2f} deg, to {tt[-1]:.1f} s", s['lost'])
                if s['lost']:
                    ax.plot(tt[-1], yy[-1], 'x', color=S.RED_MID, ms=4, mew=0.9, zorder=4)
                    ax.annotate('capsized', xy=(tt[-1], yy[-1]), xytext=(4, 0), textcoords='offset points',
                                ha='left', va='center', fontsize=8, color=S.RED_MID)
        ax.axhline(0, color=S.LIGHT, lw=0.5, zorder=0)
    label(axs[0], 'our planner', 1.0, 22, S.BLUE)
    label(axs[0], 'network, as ported', 17.5, 100, S.RED_MID, ha='right')
    label(axs[0], 'network, best gains', 60.5, 85, S.RED, ha='right')
    label(axs[1], 'our planner', 1.0, 4, S.BLUE)
    label(axs[1], 'network, best gains', 60.5, 18, S.RED, ha='right')
    axs[0].set_ylim(-140, 140)
    axs[0].set_yticks([-120, -60, 0, 60, 120])
    axs[1].set_ylim(-20, 20)
    axs[1].set_yticks([-20, -10, 0, 10, 20])
    axs[1].set_xlim(0, 61)
    axs[1].set_xticks([0, 20, 40, 60])
    axs[1].set_xlabel(r'$t$ after the hand-over (s)')
    fig.supylabel(r'Ring yaw change ($^\circ$)', fontsize=9)
    S.panel(axs[0], 'a', 'hover')
    S.panel(axs[1], 'b', r'0.4\,m/s line')
    S.save(fig, 'fig_c_yaw')


if __name__ == '__main__':
    main()
