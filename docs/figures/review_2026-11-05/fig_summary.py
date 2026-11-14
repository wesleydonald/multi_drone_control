#!/usr/bin/env python3
"""5 Nov copy of review_2026-11-02/fig_summary.py: 'quadrotors'; saves beside itself.

Error on the three references by team size, Setup 1 against simulation (fig:res_summary, redrawn 2 Nov).

One message: the real world tracks several times worse than simulation on every reference, and team size changes
little apart from the pair on the paths. Data: the real-world cells of tab:res_real (the pair: its best flight of 28 Oct, Wesley 3 Nov) against the simulated means (the same runs
as docs/figures/review_2026-10-30/fig_summary.py, read from the thesis tables). Grouped bars, red real world, blue
simulation, value above each bar. The unexplained star and the per-run dots of the 30 Oct version are gone: the tables
carry the runs, and the text says which flight differs.
5 Nov: the simulated bars are the means of feed F031 (R3489-R3506, current tree, slope, 1 m), two runs per cell.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'tools'))
import plot_style as ps  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

RED, BLUE = '#b2182b', '#2166ac'
TEXT_OF = {RED: '#7d111e', BLUE: '#174778'}     # the value labels at 70 % (Wesley 14 Nov: too light)
import json  # noqa: E402
# 13 Nov (review): the simulated bars are the means of the runs behind Table 11 (results/analysis/s41_s42_2026-11-07.json,
# R4014-R4031), read here instead of typed; the real-world bars are Table 21's single flights.
_S41 = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'results', 'analysis',
                                   's41_s42_2026-11-07.json')))


def _sim(kind):
    out = {}
    for n in (2, 3, 4):
        v = [r['err_mm'] for r in _S41 if r.get('carriers') == n and r['kind'] == kind]
        out[n] = sum(v) / len(v)
    return out


PANELS = (('Hover', {2: 25.6, 3: 21.8, 4: 21.5}, _sim('hover'), 32),
          ('Figure-eight', {2: 28.8, 3: 19.0, 4: 17.4}, _sim('figure-8'), 52),
          ('Spinning line', {2: 48.6, 3: 36.4}, _sim('spinny line'), 70))
W = 0.36


def main():
    ps.thesis()
    # Wesley 14 Nov: a team size shows only where the experiments flew it (no four on the spinning line); the narrower
    # panel keeps the bars the same width on paper
    fig, axs = plt.subplots(1, 3, figsize=(6.27, 2.25), constrained_layout=False, sharey=True,
                            gridspec_kw=dict(width_ratios=[3.2, 3.2, 2.2]))
    for ax, (title, real, sim, top), letter in zip(axs, PANELS, 'abc'):
        ns = sorted(real)
        for n in ns:
            for off, table, col in ((-W / 2, real, RED), (W / 2, sim, BLUE)):
                ax.bar(n + off, table[n], W, color=col)
                ax.annotate(f'{table[n]:.1f}', xy=(n + off, table[n]), xytext=(0, 1.5),
                            textcoords='offset points', ha='center', va='bottom', fontsize=8, color=TEXT_OF[col])
        ax.set_xticks(ns)
        ax.set_xlim(ns[0] - 0.6, ns[-1] + 0.6)
        ax.set_ylim(0, 56)
        ax.set_xlabel('Number of quadrotors')
        ax.set_title(f'({letter})\\ \\ {title}', fontsize=9, loc='left', pad=4)
    axs[0].set_ylabel('RMSE (mm)')
    fig.subplots_adjust(left=0.075, right=0.995, top=0.91, bottom=0.31, wspace=0.2)
    handles = [plt.Rectangle((0, 0), 1, 1, color=RED), plt.Rectangle((0, 0), 1, 1, color=BLUE)]
    ps.legend_below(fig, handles, ['Experiments, Setup 1', 'Simulation'], ncol=2)
    here = os.path.dirname(os.path.abspath(__file__))
    fig.savefig(os.path.join(here, 'fig_summary.pdf'))
    fig.savefig(os.path.join(here, 'fig_summary.png'), dpi=200)


if __name__ == '__main__':
    main()
