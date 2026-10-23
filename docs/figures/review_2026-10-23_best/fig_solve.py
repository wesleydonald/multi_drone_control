#!/usr/bin/env python3
"""The planner's solve time per tick on the rig laptop, on the figure-8 with 2, 3 and 4 carriers.

Message: every solve finished inside the planner's 100 ms period (longest 54, 83 and 89 ms); the median was 16, 40 and
54 ms, and the 60 ms solve budget was passed by none, 16 % and 29 % of the ticks with 2, 3 and 4 carriers.
Hero: the share of ticks solved within a time (empirical CDF), one line per fleet size in its colour
(best_style.FLEET), the 60 ms budget and the 100 ms period grey dashed. Left out: the tick spacing (the planner's ticks came
p95 123-152 ms apart, at most 144-178 ms: in the index), the twin (its wall-clock solve times are the lab PC's).
Not one variable: 2 and 3 carriers flew the 22 Oct detach mode, 4 the 7 Oct carry planner; the controlled scaling is the
S bench (first review set, fig_s_solve_time).
Data: the planner log's solve_ms over the scored path (n2_bands' window).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402



def main():
    S.setup()
    fig = S.figure('full', 2.0)
    ax = fig.subplots()
    for n in (2, 3, 4):
        f = B.Flight(B.RIG_BEST[('figure-8', n)])
        w0, w1 = B.path_parts(f)['window']
        p = f.planner()
        m = (p.t >= w0) & (p.t <= w1) & p.solve_ms.notna()
        x = np.sort(p.solve_ms[m].to_numpy(float))
        y = np.arange(1, x.size + 1) / x.size
        ax.step(x, y, where='post', color=S.FLEET_COLOUR[n], lw=S.LW_HERO)
        k = int(np.searchsorted(y, 0.5))
        ax.annotate(f'{n}', xy=(x[k], 0.5), xytext=(-5, 0), textcoords='offset points', ha='right', va='center',
                    fontsize=8, color=S.FLEET_COLOUR[n])
        tick = np.diff(p.t[(p.t >= w0) & (p.t <= w1)].to_numpy())
        print(n, key := B.RIG_BEST[('figure-8', n)], f'{x.size} ticks; solve p50 {np.median(x):.1f} p95 '
              f'{np.percentile(x, 95):.1f} max {x.max():.1f} ms; over 60 ms {100 * np.mean(x > 60):.0f} %; '
              f'tick spacing p50 {1000 * np.median(tick):.0f} p95 {1000 * np.percentile(tick, 95):.0f} '
              f'max {1000 * tick.max():.0f} ms')
    for v, text in ((60, 'budget'), (100, 'period')):
        ax.axvline(v, **S.REF)
        ax.annotate(text, xy=(v, 0.03), xytext=(3, 0), textcoords='offset points', ha='left', va='bottom', fontsize=8,
                    color=S.GREY)
    ax.annotate('carriers', xy=(0.0, 0.5), xycoords=('axes fraction', 'data'), xytext=(2, 0),
                textcoords='offset points', ha='left', va='center', fontsize=8, color=S.GREY)
    ax.set_xlim(0, 110)
    ax.set_xticks([0, 20, 40, 60, 80, 100])
    ax.set_ylim(0, 1.0)
    ax.set_yticks([0, 0.5, 1.0])
    ax.set_xlabel('Planner solve time (ms)')
    ax.set_ylabel('Share of ticks')
    S.g_ticks(ax)
    S.save(fig, 'fig_solve')


if __name__ == '__main__':
    main()
