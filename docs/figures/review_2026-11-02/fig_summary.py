#!/usr/bin/env python3
"""Error on the three references by team size, Setup 1 against simulation (fig:res_summary, redrawn 2 Nov).

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
PANELS = (('Hover, position error', {2: 25.6, 3: 21.8, 4: 21.5}, {2: 1.1, 3: 1.05, 4: 1.35}, 32),
          ('Figure-eight, contour error', {2: 28.8, 3: 19.0, 4: 17.4}, {2: 8.35, 3: 10.7, 4: 12.85}, 52),
          ('Spinny line, contour error', {2: 48.6, 3: 36.4}, {2: 8.15, 3: 5.75, 4: 4.5}, 70))
W = 0.36


def main():
    ps.thesis()
    fig, axs = plt.subplots(1, 3, figsize=(6.27, 2.25), constrained_layout=False)
    for ax, (title, real, sim, top), letter in zip(axs, PANELS, 'abc'):
        for n in (2, 3, 4):
            for off, table, col in ((-W / 2, real, RED), (W / 2, sim, BLUE)):
                if n in table:
                    ax.bar(n + off, table[n], W, color=col)
                    ax.annotate(f'{table[n]:.1f}', xy=(n + off, table[n]), xytext=(0, 1.5),
                                textcoords='offset points', ha='center', va='bottom', fontsize=7, color=col)
                else:
                    ax.annotate('Not flown', xy=(n + off, 0), xytext=(0, 2), textcoords='offset points',
                                ha='center', va='bottom', fontsize=6.5, color='0.45', rotation=90)
        ax.set_xticks((2, 3, 4))
        ax.set_xlim(1.4, 4.6)
        ax.set_ylim(0, top)
        ax.set_xlabel('Number of carriers')
        ax.set_title(f'({letter}) {title}', fontsize=8, loc='left', pad=3)
    axs[0].set_ylabel('Error (mm)')
    fig.subplots_adjust(left=0.075, right=0.995, top=0.91, bottom=0.31, wspace=0.2)
    handles = [plt.Rectangle((0, 0), 1, 1, color=RED), plt.Rectangle((0, 0), 1, 1, color=BLUE)]
    ps.legend_below(fig, handles, ['Real world, Setup 1', 'Simulation'], ncol=2)
    fig.savefig('fig_summary.pdf')
    fig.savefig('fig_summary.png', dpi=200)


if __name__ == '__main__':
    main()
