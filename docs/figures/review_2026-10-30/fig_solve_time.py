#!/usr/bin/env python3
"""The planner's solve time per tick against the number of carriers, as flown at five SQP iterations per tick.

Message: the solve grows with the team. On the ground computer, in flight, every solve finished inside the 100 ms
period; the median was 16, 34 and 56 ms with two, three and four carriers, and with four a third of the solves ran
past the 60 ms budget. In simulation, on a different computer, the median grew to 210 ms with eight.
Hero: (a) the real-world distributions (violins), the 60 ms budget and the 100 ms period dashed grey. (b) the same in
simulation, smaller and in the other colour: a different computer, no budget, and slower than real time.
Left out: the rest of the tick (references, publishing, logging: in index.md), the 22 Oct flights (an older code
tree, 0.55 m cables), ticks without a solve (log rows whose `published` is not 'solved' repeat the last solve time:
after touchdown and before the lift), the budget line in (b) (the simulation has none). Each violin spans all its solves.
Data: the planner log (logs/dissipative_planner/*/log.csv), phase 'planner', published 'solved', solve_ms grouped by
its column n. Real world: 28 Oct (pair) and 29 Oct (triangle, four) flights, tree 413ec0e (the planner unchanged in
the working copy). Simulation: lab batch 18 (R2764-R2779), tree week7 63b504f, two runs at a time, RTF 0.22-0.25.
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from review30_style import S, TEXT  # noqa: E402

RIG = os.path.join(S.REPO, 'results', 'rig')
REAL = {2: [('2026-10-28', f) for f in ('f2a', 'ls2a', 'ls2b_4', 'f2b', 'f2c')],
        3: [('2026-10-29', f) for f in ('sq2_5', 'sq3', 'sq4_5', 'sq5_2', 'k1_4')],
        4: [('2026-10-29', f) for f in ('h4', 'h4f_3', 'f4')]}
SIM = [f'R{r}' for r in range(2764, 2780)]
BUDGET_MS, PERIOD_MS = 60.0, 100.0


def solved(path):
    d = pd.read_csv(path, low_memory=False)
    return d[(d.phase == 'planner') & (d.published == 'solved')]


def load_real():
    out = {}
    for n, flights in REAL.items():
        parts = []
        for day, name in flights:
            (path,) = glob.glob(os.path.join(RIG, day, f'{name}_logs', 'logs', 'dissipative_planner', '*', 'log.csv'))
            p = solved(path)
            assert (p.n == n).all(), (name, sorted(p.n.unique()))
            parts.append(p)
        out[n] = pd.concat(parts)
    return out


def load_sim():
    out = {}
    for rid in SIM:
        (path,) = glob.glob(os.path.join(S.REPO, 'results', '2026-10-2*', f'{rid}_*', 'logs', 'dissipative_planner',
                                         '*', 'log.csv'))
        for n, g in solved(path).groupby('n'):
            out.setdefault(int(n), []).append(g)
    return {n: pd.concat(v) for n, v in sorted(out.items())}


def summary(label, groups):
    print(f'{label}: n, ticks, solve p50 / p95 / max (ms), over {BUDGET_MS:.0f} ms, cut short (<5 SQP), '
          f'tick p50 / p95 (ms), tick over {PERIOD_MS:.0f} ms')
    for n, g in groups.items():
        s, t = g.solve_ms.to_numpy(float), g.tick_ms.to_numpy(float)
        print(f'  {n}  {s.size:5d}  {np.median(s):6.1f} {np.percentile(s, 95):6.1f} {s.max():6.1f}  '
              f'{100 * np.mean(s > BUDGET_MS):5.1f} %  {100 * np.mean(g.sqp_calls < 5):4.1f} %  '
              f'{np.median(t):6.1f} {np.percentile(t, 95):6.1f}  {100 * np.mean(t > PERIOD_MS):4.1f} %')


def tint(colour, a):
    """colour mixed with white: a = 1 is the colour, 0 is white."""
    rgb = np.array(S.plt.matplotlib.colors.to_rgb(colour))
    return tuple(1.0 - a * (1.0 - rgb))


def violins(ax, groups, colour, half_width=0.36):
    """One violin per team size over all its solves, shortest to longest: a Gaussian kernel density (Scott's
    bandwidth) scaled to the same widest half-width for every team size; the median as a bar across it, its value
    beside it."""
    from scipy.stats import gaussian_kde
    for n, g in groups.items():
        s = g.solve_ms.to_numpy(float)
        y = np.linspace(s.min(), s.max(), 400)
        dens = gaussian_kde(s)(y)
        w = half_width * dens / dens.max()
        ax.fill_betweenx(y, n - w, n + w, facecolor=tint(colour, 0.3), edgecolor=colour, lw=0.7, zorder=2)
        med = float(np.median(s))
        wm = float(np.interp(med, y, w))
        ax.plot([n - wm, n + wm], [med, med], color=colour, lw=S.LW_HERO, solid_capstyle='butt', zorder=3)
        ax.annotate(f'{med:.0f}', xy=(n + w.max(), med), xytext=(2.5, 0), textcoords='offset points', ha='left',
                    va='center', fontsize=8, color=colour)


def ref(ax, y, text, x, ha):
    ax.axhline(y, **S.REF)
    ax.annotate(text, xy=(x, y), xycoords=('axes fraction', 'data'), xytext=(2 if ha == 'left' else -2, 1.5),
                textcoords='offset points', ha=ha, va='bottom', fontsize=8, color=TEXT)


def main():
    real, sim = load_real(), load_sim()
    summary('real world', real)
    summary('simulation', sim)

    S.setup()
    fig = S.figure('full', 2.5)
    gs = fig.add_gridspec(1, 2, width_ratios=[3.6, 4.0])
    ax_r, ax_s = fig.add_subplot(gs[0]), fig.add_subplot(gs[1])

    violins(ax_r, real, S.BLUE)
    violins(ax_s, sim, S.RED)

    # (a) the budget and the period of the ground computer; (b) the period only (no budget in simulation, and its
    # period is simulated time)
    ref(ax_r, BUDGET_MS, '60 ms budget', 0.0, 'left')
    ref(ax_r, PERIOD_MS, '100 ms period', 0.0, 'left')
    ref(ax_s, PERIOD_MS, '100 ms period', 1.0, 'right')
    s4 = real[4].solve_ms.to_numpy(float)
    ax_r.annotate(f'{100 * np.mean(s4 > BUDGET_MS):.0f}\\,\\% over budget', xy=(4, s4.max()), xytext=(0, 3),
                  textcoords='offset points', ha='center', va='bottom', fontsize=8, color=S.BLUE)

    ax_r.set_xlim(1.4, 4.75)
    ax_r.set_xticks([2, 3, 4])
    ax_r.set_ylim(0, 117)
    ax_r.set_yticks([0, 20, 40, 60, 80, 100])
    ax_s.set_xlim(3.4, 8.9)
    ax_s.set_xticks([4, 5, 6, 7, 8])
    ax_s.set_ylim(0, 520)
    ax_s.set_yticks([0, 100, 200, 300, 400, 500])
    for ax in (ax_r, ax_s):
        ax.set_ylabel('Solve time (ms)')
        ax.set_xlabel('Number of carriers')
    S.panel(ax_r, 'a', 'Real world')
    S.panel(ax_s, 'b', 'Simulation')
    S.save(fig, 'fig_solve_time')


if __name__ == '__main__':
    main()
