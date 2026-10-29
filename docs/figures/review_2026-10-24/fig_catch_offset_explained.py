#!/usr/bin/env python3
"""U, the magnet caught off its slot, with a drawing of the two offset directions (Wesley 27 Oct: make 'along the rim'
and 'radially inward' obvious). Panels (b) and (c) are review_2026-10-23/fig_u_catch_offset's data and look; (a) is a
top view of the ring near the newcomer's plate: the slot, the next plate 11.6 cm along the rim, the centre inward.
Data: R2629-R2684, R2727-R2732 through tools/catch_offset.analyse (the 23 Oct figure's cache).
"""
import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402
import catch_offset as C  # noqa: E402
from matplotlib.patches import Rectangle, Wedge, FancyArrowPatch  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    'u23', os.path.join(os.path.dirname(HERE), 'review_2026-10-23', 'fig_u_catch_offset.py'))
U = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(U)

R_PLATE = 0.225                    # m, plate centres
STEP = np.radians(30.0)            # plates every 30 deg: the next plate 11.6 cm along the rim


def drawing(ax):
    ax.set_aspect('equal')
    ax.add_patch(Wedge((0, 0), 0.258, 55, 125, width=0.066, facecolor=S.FILL, edgecolor=S.LIGHT, lw=0.6))
    for k, ang in enumerate((np.pi / 2, np.pi / 2 - STEP, np.pi / 2 + STEP)):
        x, y = R_PLATE * np.cos(ang), R_PLATE * np.sin(ang)
        ax.add_patch(Rectangle((x - 0.03, y - 0.03), 0.06, 0.06, facecolor='white',
                               edgecolor=S.INK if k == 0 else S.LIGHT, lw=0.9 if k == 0 else 0.6, zorder=2))
    sx, sy = 0.0, R_PLATE
    ax.plot(sx, sy, 'o', ms=4.5, color=S.INK, zorder=5)
    ax.annotate("newcomer's slot", xy=(sx, sy + 0.03), xytext=(0, 4), textcoords='offset points', ha='center',
                va='bottom', fontsize=8, color=S.INK)
    nx, ny = R_PLATE * np.cos(np.pi / 2 - STEP), R_PLATE * np.sin(np.pi / 2 - STEP)
    ax.annotate('next plate', xy=(nx, ny - 0.03), xytext=(0, -3), textcoords='offset points', ha='center',
                va='top', fontsize=8, color=S.GREY)
    ax.add_patch(FancyArrowPatch((sx + 0.008, sy - 0.001), (nx - 0.006, ny + 0.004),
                                 connectionstyle='arc3,rad=-0.12', arrowstyle='-|>', mutation_scale=10,
                                 color=S.BLUE_LIGHT if False else S.INK, lw=1.1, zorder=6))
    ax.annotate('along the rim', xy=((sx + nx) / 2 + 0.01, (sy + ny) / 2 + 0.012), ha='left', va='bottom',
                fontsize=8, color=S.INK)
    ax.add_patch(FancyArrowPatch((sx, sy - 0.008), (sx, 0.085), arrowstyle='-|>', mutation_scale=10, color=S.INK,
                                 lw=1.1, zorder=6))
    ax.annotate('radially inward\n(towards the ring centre)', xy=(0.010, 0.105), ha='left', va='center',
                fontsize=8, color=S.INK)
    ax.set_xlim(-0.10, 0.17)
    ax.set_ylim(0.075, 0.29)
    ax.axis('off')


def main():
    rows = U.runs()
    S.setup()
    fig = S.figure('full', 2.5)
    gs = fig.add_gridspec(1, 3, width_ratios=[0.95, 1, 1], wspace=0.22)
    a = fig.add_subplot(gs[0])
    axs = [fig.add_subplot(gs[1]), None]
    axs[1] = fig.add_subplot(gs[2], sharey=axs[0])
    drawing(a)
    arms = (('slot', S.RED, 's', 'slot assumed'), ('measured', S.BLUE, 'o', 'measured catch point'))
    for ax, direction, xlabel in ((axs[0], 't', 'Offset along the rim (cm)'),
                                  (axs[1], 'ri', 'Offset radially inward (cm)')):
        S.ref_line(ax, C.FOLD_TILT_BAR_DEG, r'$5^\circ$ bar' if direction == 'ri' else None)
        for arm, col, mk, text in arms:
            mine = [r for r in rows if r['arm'] == arm and r['direction'] in ('d0', direction)
                    and r['verdict'] != 'VOID' and r['run'] not in U.VOID_AS_CARD]
            ok = [r for r in mine if not r['failures'] and r['tilt'] is not None]
            ds = sorted({float(r['d']) for r in ok} - {11.6})
            mean = [np.mean([r['tilt'] for r in ok if float(r['d']) == d]) for d in ds]
            ax.plot(ds, mean, color=col, lw=S.LW_HERO, zorder=3)
            ax.plot([float(r['d']) for r in ok], [r['tilt'] for r in ok], mk, ms=S.MS if mk == 'o' else S.MS - 0.4,
                    mfc=col, mec='white', mew=0.4, zorder=4)
            if direction == 'ri':
                at = 12.0 if arm == 'slot' else 14.0
                y = float(np.interp(at, ds, mean))
                ax.annotate(text, xy=(at, y), xytext=(-6, 4) if arm == 'slot' else (0, 5),
                            textcoords='offset points', ha='right' if arm == 'slot' else 'center',
                            va='bottom', fontsize=8, color=col)
        top = 20 if direction == 'ri' else 16
        ax.set_xticks(list(range(0, top + 1, 4)))
        ax.set_xlim(-0.8, top + 0.8)
        ax.set_xlabel(xlabel)
    axs[0].set_ylabel(r'Fold-in ring tilt peak ($^\circ$)')
    axs[0].set_ylim(0, 21)
    axs[0].set_yticks([0, 5, 10, 15, 20])
    plt_hide = axs[1].get_yticklabels()
    for lab in plt_hide:
        lab.set_visible(False)
    S.panel(a, 'a')
    S.panel(axs[0], 'b')
    S.panel(axs[1], 'c')
    S.save(fig, 'fig_catch_offset_explained')


if __name__ == '__main__':
    main()
