#!/usr/bin/env python3
"""Tracking error across fleet size and motion: the best flight of each cell, rig beside twin.

Message: on the rig the ring keeps within 14-36 mm of its path on every motion flown with 2, 3 or 4 carriers, and
within 22 mm of its hover point; the spin (the circle with one yaw turn) is the hardest motion (45 and 86 mm). The twin
(1 m cables) keeps within 1-10 mm in every cell, with no trend across 2-4 carriers.
Two designs (Wesley picks):
  fig_summary        (a) rig, (b) twin: one column per motion, each fleet size's error drawn as its digit in its colour,
                     on a log axis (the cells span 1 to 86 mm).
  fig_summary_grid   the same numbers as a grid of cells (motion x carriers), darker = larger (log scale), one scale.
Numbers: best_data.score (n2_bands' 22 Oct rule: clock-free on a path, distance to the hold point in a hover). The
flights: best_data.RIG_BEST and TWIN_GRID; the panel names take the cable lengths from the flights. Left out: the lag
(fig_components), the tilt (fig_tilt).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402

MOTIONS = ('hover', 'circle', 'figure-8', 'line', 'spinny line', 'spin')
DX = 0.24                         # a fleet size's offset inside its motion's column


def cells(table):
    out, cable = {}, set()
    for (m, n), k in table.items():
        if m in MOTIONS:
            f = B.Flight(k)
            out[(m, n)] = 1000 * B.score(f)['err']
            cable.add(f.cable())
    return out, cable


def name(source, cable):
    kind = 'rods' if source == 'rig' and max(cable) < 0.8 else 'cables'
    return f'{source}, ' + ' and '.join(f'{c:g}' for c in sorted(cable)) + rf'\,m {kind}'


def digits(ax, val):
    for i, motion in enumerate(MOTIONS):
        pts = sorted((n, v) for (m, n), v in val.items() if m == motion)
        if not pts:
            continue
        x = np.array([i + (n - 3) * DX for n, _ in pts])
        ax.plot(x, [v for _, v in pts], color=S.LIGHT, lw=S.LW_CTX, zorder=1)
        for xx, (n, v) in zip(x, pts):
            ax.text(xx, v, str(n), ha='center', va='center', fontsize=9, color=S.FLEET_COLOUR[n], zorder=3,
                    bbox=dict(boxstyle='square,pad=0.08', fc='white', ec='none'))


def grid(ax, val, norm, cmap):
    from matplotlib.patches import Rectangle
    top = len(MOTIONS) - 1
    for i, motion in enumerate(MOTIONS):
        for j, n in enumerate((2, 3, 4)):
            v = val.get((motion, n))
            if v is None:
                continue
            ax.add_patch(Rectangle((j - 0.47, top - i - 0.42), 0.94, 0.84, color=cmap(norm(v)), lw=0))
            ax.text(j, top - i, f'{v:.0f}' if v >= 10 else f'{v:.1f}', ha='center', va='center', fontsize=8,
                    color='white' if norm(v) > 0.6 else S.INK)


def main():
    rig, rig_cable = cells(B.RIG_BEST)
    twin, twin_cable = cells(B.TWIN_GRID)
    for label, val in (('rig', rig), ('twin', twin)):
        print(label, '  '.join(f'{m} {n}: {v:.1f}' for (m, n), v in sorted(val.items())))
    names = (name('rig', rig_cable), name('twin', twin_cable))

    # design A: digits on a log axis
    S.setup()
    fig = S.figure('full', 2.3)
    axs = fig.subplots(1, 2, sharey=True)
    for ax, val, letter, text in ((axs[0], rig, 'a', names[0]), (axs[1], twin, 'b', names[1])):
        digits(ax, val)
        ax.set_xlim(-0.5, len(MOTIONS) - 0.5)
        ax.set_xticks(range(len(MOTIONS)), [m.replace(' ', '\n') for m in MOTIONS])
        ax.tick_params(axis='x', length=0)
        S.panel(ax, letter, text)
    axs[0].set_yscale('log')
    axs[0].set_ylim(0.7, 130)
    axs[0].set_yticks([1, 3, 10, 30, 100])
    axs[0].minorticks_off()
    axs[0].annotate('digit = carriers', xy=(1, 1), xycoords='axes fraction', xytext=(0, 4), textcoords='offset points',
                    ha='right', va='bottom', fontsize=8, color=S.GREY)
    axs[0].set_ylabel('Tracking error (mm)')
    S.g_ticks(axs[0], which='y')
    S.save(fig, 'fig_summary')

    # design B: the same numbers as a grid, darker = larger on a log scale
    import matplotlib.colors as mc
    cmap = mc.LinearSegmentedColormap.from_list('teal', ['#ffffff', '#1b7a6e'])
    norm = mc.LogNorm(vmin=0.7, vmax=100)
    S.setup()
    fig = S.figure('full', 2.5)
    axs = fig.subplots(1, 2, sharey=True)
    for ax, val, letter, text in ((axs[0], rig, 'a', names[0]), (axs[1], twin, 'b', names[1])):
        grid(ax, val, norm, cmap)
        ax.set_xlim(-0.5, 2.5)
        ax.set_ylim(-0.5, len(MOTIONS) - 0.5)
        ax.set_xticks([0, 1, 2], ['2', '3', '4'])
        for t, n in zip(ax.get_xticklabels(), (2, 3, 4)):
            t.set_color(S.FLEET_COLOUR[n])
        ax.set_xlabel('Carriers')
        ax.tick_params(length=0)
        for side in ('left', 'bottom'):
            ax.spines[side].set_visible(False)
        S.panel(ax, letter, text)
    axs[0].set_yticks(range(len(MOTIONS) - 1, -1, -1), list(MOTIONS))
    S.save(fig, 'fig_summary_grid')


if __name__ == '__main__':
    main()
