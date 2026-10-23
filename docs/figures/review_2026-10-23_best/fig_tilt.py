#!/usr/bin/env python3
"""The ring's tilt on the rig's figure-8 with 2, 3 and 4 carriers.

Message: with two carriers the ring rocks between about 1 and 14 deg (mean 7 deg) about the pair's line, a rotation
no cable resists; with three or four it stays under 6 deg (mean 2-3 deg).
Two designs (Wesley picks):
  fig_tilt_box     the three flights as boxes in their fleet colours (quartiles; whiskers 5-95 %; the peak as a dot).
  fig_tilt         the tilt along the eight, one line per fleet size in its colour.
Left out: the tilt's direction, the lift and the landing, the twin (under 1.2 deg on the same eight at 1 m: in the
index). Flights: best_data.RIG_BEST's figure-8 cells, as fig_traj.
"""
import math
import os
import sys

import numpy as np
from matplotlib.patches import Rectangle

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402


def main():
    S.setup()
    fig = S.figure('full', 1.9)
    ax = fig.subplots()
    ends, tilts = {}, {}
    for n in (4, 3, 2):
        f = B.Flight(B.RIG_BEST[('figure-8', n)])
        s = B.path_parts(f)
        tilt = f.run().payload_tilt_deg.to_numpy()
        pos = s['s_at'] - s['s_at'][np.argmax(s['w'])]
        L = float(s['path'][1][-1])
        w = s['w'] & (pos <= L - 0.1)            # the last 10 cm: the ring is projected back across the crossing
        o = np.argsort(pos[w])
        ax.plot(pos[w][o], tilt[w][o], **S.FLEET[n])
        ends[n] = (pos[w][o][-1], tilt[w][o][-1])
        tilts[n] = tilt[w]
        print(n, f'tilt mean {np.nanmean(tilt[w]):.2f}, median {np.nanmedian(tilt[w]):.2f}, '
                 f'p95 {np.nanpercentile(tilt[w], 95):.2f}, max {np.nanmax(tilt[w]):.2f} deg')
    for n, (x, y) in ends.items():
        ax.annotate(f'{n} carriers' if n == 2 else f'{n}', xy=(x, y), xytext=(4, 0), textcoords='offset points',
                    ha='left', va='center', fontsize=8, color=S.FLEET_COLOUR[n])
    top = 5 * math.ceil(max(np.nanmax(v) for v in tilts.values()) / 5)
    ax.set_xlim(0, 1.08 * max(x for x, _ in ends.values()))
    ax.set_ylim(0, top)
    ax.set_yticks(range(0, top + 1, 5))
    ax.set_xlabel('Ring position along the figure-8 (m)')
    ax.set_ylabel(r'Ring tilt ($^\circ$)')
    S.save(fig, 'fig_tilt')

    S.setup()
    fig = S.figure('full', 1.5)
    ax = fig.subplots()
    for n in (2, 3, 4):
        x = tilts[n][np.isfinite(tilts[n])]
        c = S.FLEET_COLOUR[n]
        p5, q1, med, q3, p95 = np.percentile(x, [5, 25, 50, 75, 95])
        y = 4 - n
        ax.plot([p5, q1], [y, y], color=c, lw=S.LW_CTX)
        ax.plot([q3, p95], [y, y], color=c, lw=S.LW_CTX)
        ax.add_patch(Rectangle((q1, y - 0.18), q3 - q1, 0.36, fc=S.tint(c, 0.3), ec=c, lw=S.LW_CTX))
        ax.plot([med, med], [y - 0.18, y + 0.18], color=c, lw=S.LW_HERO)
        ax.plot(np.max(x), y, 'o', ms=2.6, color=c)
    ax.set_yticks([2, 1, 0], ['2 carriers', '3', '4'])
    for t, n in zip(ax.get_yticklabels(), (2, 3, 4)):
        t.set_color(S.FLEET_COLOUR[n])
    ax.tick_params(axis='y', length=0)
    ax.set_ylim(-0.6, 2.6)
    ax.set_xlim(0, top)
    ax.set_xticks(range(0, top + 1, 5))
    ax.set_xlabel(r'Ring tilt on the figure-8 ($^\circ$)')
    S.save(fig, 'fig_tilt_box')


if __name__ == '__main__':
    main()
