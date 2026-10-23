#!/usr/bin/env python3
"""R, the re-fly at 1 m: the one carry cell that left the envelope, the 3-quad orbit at 0.3 m/s
(docs/analysis/refly_1m_table_2026-10-23.md, T7t and its attribution re-fly).

Message: at 1 m the orbit's clock-free path error crosses the 25 mm envelope (25.7 / 25.9 mm); most of the rise came
with the code tree (0.55 m: 15.2 / 13.2 mm on the 21 Oct tree, 23.7 / 22.7 on the 23 Oct tree), the longer cable adds
about 2.6 mm.
Hero: the clock-free path error (tools/n2_bands.path_metrics_clock_free, as tools/refly_score.py scores T7t) against
the cable length, the 21 Oct tree red and the 23 Oct tree blue, one marker per run; the envelope dashed. Left out: the
radius ratio (the same story: 0.975 / 0.977, 0.954 / 0.956, 0.950 / 0.950 against 0.95), tilt, the other 14 cells
(inside at both lengths).
Runs: R1871, R1872 (0.55 m, 21 Oct tree); R2580, R2581 (0.55 m, 23 Oct tree); R2521, R2536 (1 m, 23 Oct tree).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import refly_score as R  # noqa: E402

ENVELOPE_MM = 25.0       # T7t's clock-free bar (the carry card)
OLD = {0.55: ('R1871', 'R1872')}
NEW = {0.55: ('R2580', 'R2581'), 1.0: ('R2521', 'R2536')}


def main():
    S.setup()
    fig = S.figure('half', 2.3)
    ax = fig.subplots()
    S.ref_line(ax, ENVELOPE_MM, r'envelope', va='top')
    for runs, col, text, dx in ((OLD, S.RED, '21 Oct code', -0.012), (NEW, S.BLUE, '23 Oct code', 0.012)):
        xs, means = [], []
        for L, rids in sorted(runs.items()):
            v = [R.m_carry(r)['cf_mm'] for r in rids]
            print(text, L, dict(zip(rids, [f'{x:.2f}' for x in v])))
            ax.plot([L + dx] * len(v), v, 'o', ms=S.MS, mfc=col, mec='white', mew=0.5, zorder=3)
            xs.append(L + dx)
            means.append(float(np.mean(v)))
        if len(xs) > 1:
            ax.plot(xs, means, color=col, lw=S.LW_CTX, zorder=2)
        ax.annotate(text, xy=(xs[0], means[0]), xytext=(-7, 0), textcoords='offset points', ha='right',
                    va='center', fontsize=8, color=col)
    ax.set_xlim(0.3, 1.12)
    ax.set_xticks([0.55, 1.0])
    ax.set_xticklabels(['0.55', '1'])
    ax.spines['bottom'].set_bounds(0.55, 1.0)
    ax.set_ylim(0, 30)
    ax.set_yticks([0, 10, 20, 30])
    ax.set_xlabel(r'Cable length (m)')
    ax.set_ylabel(r'Clock-free path error (mm)')
    S.save(fig, 'fig_refly_orbit', width='half')


if __name__ == '__main__':
    main()
