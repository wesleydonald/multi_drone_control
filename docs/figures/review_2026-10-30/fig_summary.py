#!/usr/bin/env python3
"""Path error by trajectory and team size, real world (earlier setup) beside simulation (redesign of Figure 12,
fig:res_summary; replaces review_2026-10-23_best/fig_summary's digits on a log axis over six paths).

Message: the real world tracks several times worse than simulation on every trajectory, and the team size changes
little (apart from the pair on the paths).
Hero: one panel per trajectory (hover, figure-eight, spinny line), grouped bars per team size, real world red beside
simulation blue, each bar's value on it; linear y, each panel its own scale.
Metric (the 4.1 tables tab:res_hover / tab:res_fig8 / tab:res_spin): hover, the position error (rms 3-D distance to the
hold point, from 2 s after the reference tops out to LAND); the paths, the clock-free path error (rms of the distance
across the path and in height, without the clock). best_data.score (the n2_bands rule), a cell the mean over its runs.
Left out: the circle, the line and the spin (Appendix), the later setup (not all cells flown), lag and tilt (tables).
Data: the run lists of the 4.1 source comment (results_discussion.tex, 31 Oct), below.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from review30_style import S  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'review_2026-10-23_best'))
import best_data as B  # noqa: E402

TRAJ = (('hover', 'Hover, position error'), ('fig8', 'Figure-eight, path error'),
        ('spin', 'Spinny line, path error'))
REAL = {('hover', 2): ['2026-10-28/hS2'], ('hover', 3): ['2026-10-22/h3'], ('hover', 4): ['2026-10-07/r20004'],
        ('fig8', 2): ['2026-10-28/f2a', '2026-10-28/f2b', '2026-10-28/f2c'], ('fig8', 3): ['2026-10-22/f3h'],
        ('fig8', 4): ['2026-10-07/r2008e60'],
        ('spin', 2): ['2026-10-28/ls2a', '2026-10-28/ls2b_4'], ('spin', 3): ['2026-10-22/ls3c']}
SIM = {('hover', 2): ['R2330', 'R2331'], ('hover', 3): ['R2337', 'R2338'], ('hover', 4): ['R2349', 'R2350'],
       ('fig8', 2): ['R2334', 'R2335'], ('fig8', 3): ['R2341', 'R2342'], ('fig8', 4): ['R2353', 'R2354'],
       ('spin', 2): ['R2362', 'R2363'], ('spin', 3): ['R2345', 'R2346'], ('spin', 4): ['R2357', 'R2358']}
MARK = {('fig8', 4): '*'}                 # the table's footnote a: half the speed, an earlier code tree
W = 0.36                                  # bar width; the pair of bars spans 2 W


def cells(table):
    """Each cell's runs (mm); the bar is their mean."""
    return {k: [1000 * B.score(B.Flight(r))['err'] for r in runs] for k, runs in table.items()}


def label(ax, x, v, colour, mark='', y=None):
    """The bar's value v written above y (the bar's top, or its highest run)."""
    ax.annotate(f'{v:.1f}{mark}' if v < 100 else f'{v:.0f}{mark}', xy=(x, v if y is None else y), xytext=(0, 1.5),
                textcoords='offset points', ha='center', va='bottom', fontsize=7.5, color=colour)


def main():
    real, sim = cells(REAL), cells(SIM)
    for name, val in (('real world', real), ('simulation', sim)):
        print(name, '  '.join(f'{m} {n}: {np.mean(v):.1f} ({len(v)})' for (m, n), v in sorted(val.items())))

    S.setup()
    fig = S.figure('full', 2.45)
    axs = fig.subplots(1, 3)
    top = {'hover': 32, 'fig8': 64, 'spin': 72}
    for ax, (m, title), letter in zip(axs, TRAJ, 'abc'):
        for n in (2, 3, 4):
            for dx, val, colour in ((-W / 2, real, S.RED), (W / 2, sim, S.BLUE)):
                runs = val.get((m, n))
                if runs is None:
                    ax.annotate('not flown', xy=(n + dx, 0), xytext=(0, 3), textcoords='offset points',
                                ha='center', va='bottom', fontsize=7, color=S.GREY, rotation=90)
                    continue
                v = float(np.mean(runs))
                odd = val is real and (m, n) in MARK          # another speed and tree: the light shade
                ax.bar(n + dx, v, W, color=S.RED_LIGHT if odd else colour, lw=0, zorder=2)
                if len(runs) > 1:                             # each run, on its bar
                    ax.plot([n + dx] * len(runs), runs, 'o', ms=2.2, mfc='white', mec=S.INK, mew=0.5, zorder=3)
                label(ax, n + dx, v, colour, MARK.get((m, n), '') if val is real else '', y=max(v, max(runs)))
        ax.set_xticks([2, 3, 4])
        ax.set_xlim(1.45, 4.55)
        ax.set_ylim(0, top[m])
        ax.set_xlabel('Number of carriers')
        S.panel(ax, letter, title)
    axs[0].set_ylabel('Error (mm)')
    handles = [S.plt.Rectangle((0, 0), 1, 1, color=S.RED, lw=0), S.plt.Rectangle((0, 0), 1, 1, color=S.BLUE, lw=0)]
    handles.append(S.plt.Line2D([], [], ls='none', marker='o', ms=3, mfc='white', mec=S.INK, mew=0.5))
    S.ps.legend_below(fig, handles, ['real world, earlier setup (0.55 m cables)', 'simulation (1 m links)', 'one run'],
                      ncol=3)
    S.save(fig, 'fig_summary')


if __name__ == '__main__':
    main()
