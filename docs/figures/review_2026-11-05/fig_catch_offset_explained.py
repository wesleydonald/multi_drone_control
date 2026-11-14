#!/usr/bin/env python3
"""5 Nov revision (Wesley: "extremely high quality"): the drawing redone (nothing clipped, no label on a plate,
(a) aligned with (b) and (c)), the thesis' words (plate centre, measured catch point, ring tilt peak).

U, the magnet caught off its slot, with a drawing of the two offset directions (Wesley 27 Oct: make 'along the rim'
and 'radially inward' obvious). Panels (b) and (c) are review_2026-10-23/fig_u_catch_offset's data and look; (a) is a
top view of the ring near the newcomer's plate: the slot, the next plate 11.6 cm along the rim, the centre inward.
Data: R2629-R2684, R2727-R2732 through tools/catch_offset.analyse (the 23 Oct figure's cache).
"""
import importlib.util
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'review_2026-10-24'))
import review_style as S  # noqa: E402
S.HERE = HERE                      # save beside this 5 Nov copy, not into the 24 Oct folder
import catch_offset as C  # noqa: E402
from matplotlib.patches import Rectangle, Wedge, FancyArrowPatch  # noqa: E402

_spec = importlib.util.spec_from_file_location(
    'u23', os.path.join(os.path.dirname(HERE), 'review_2026-10-23', 'fig_u_catch_offset.py'))
U = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(U)

R_PLATE = 0.225                    # m, plate centres
STEP = np.radians(30.0)            # plates every 30 deg: the next plate 11.6 cm along the rim


VARIANT = os.environ.get('FIG21_VARIANT', 'photo')  # 'photo' (his pick, 14 Nov), 'drawn', 'notarget' (drawn, no label)
RING_PHOTO = os.path.expanduser('~/thesis_full_draft/Images/setup1_ring_sq.jpg')   # the ring from above (Fig 7a)
BOX = dict(boxstyle='round,pad=0.25', fc='white', ec='none', alpha=0.9)


def photo_drawing(ax):
    """The two directions drawn on a photo of the ring from above (Wesley 14 Nov: white arrows with a black border,
    labels on white with a black border). The 900 px photo is placed so its ring (outer edge 411 px from the centre)
    has the true 0.255 m radius; the plates sit at 0.236 m in it."""
    import matplotlib.image as mpimg
    import matplotlib.patheffects as pe
    from matplotlib.path import Path
    img = mpimg.imread(RING_PHOTO)
    k = 0.255 / 411.0
    h, w = img.shape[:2]
    ax.imshow(img, extent=(-w / 2 * k, w / 2 * k, -h / 2 * k, h / 2 * k), zorder=0)
    r = 0.236
    th = np.linspace(np.pi / 2 - 0.15, np.pi / 2 - STEP + 0.12, 24)
    border = [pe.Stroke(linewidth=3.2, foreground='black'), pe.Normal()]
    for verts in (np.column_stack([r * np.cos(th), r * np.sin(th)]), np.array([[0.0, r - 0.03], [0.0, 0.07]])):
        ax.add_patch(FancyArrowPatch(path=Path(verts), arrowstyle='-|>', mutation_scale=10, color='white', lw=1.4,
                                     path_effects=border, zorder=6))
    box = dict(boxstyle='round,pad=0.25', fc='white', ec='black', lw=0.6)
    ax.annotate('Along rim', xy=(0.135, 0.252), ha='left', va='center', fontsize=8, color='black', bbox=box, zorder=7)
    ax.annotate('Radially\ninward', xy=(0, 0.045), ha='center', va='top', fontsize=8, color='black', linespacing=1.1,
                bbox=box, zorder=7)
    ax.set_xlim(-w / 2 * k, w / 2 * k)
    ax.set_ylim(-h / 2 * k, h / 2 * k)
    ax.set_aspect('equal')
    ax.set_anchor('N')
    ax.axis('off')


def drawing(ax):
    """Top view of the whole ring: plate 9 (the target) in black, its 55 mm plates on a 0.225 m radius, and the two
    directions the catch point was moved, towards the next plate along the rim and towards the ring's centre."""
    if VARIANT == 'photo':
        return photo_drawing(ax)
    from matplotlib.patches import Circle
    ax.set_aspect('equal')
    ax.add_patch(Wedge((0, 0), 0.255, 0, 360, width=0.065, facecolor=S.FILL, edgecolor=S.LIGHT, lw=0.6))
    for k in range(12):
        ang = np.pi / 2 - k * STEP
        ax.add_patch(Circle((R_PLATE * np.cos(ang), R_PLATE * np.sin(ang)), 0.0275, facecolor='white',
                            edgecolor=S.INK if k == 0 else S.LIGHT, lw=1.0 if k == 0 else 0.6, zorder=2))
    sx, sy = 0.0, R_PLATE
    ax.plot(sx, sy, 'o', ms=3.0, color=S.INK, zorder=5)
    if VARIANT != 'notarget':
        ax.annotate('Target plate', xy=(sx - 0.0275, sy + 0.01), xytext=(-3, 3), textcoords='offset points', ha='right',
                    va='bottom', fontsize=8, color=S.INK)
    th = np.linspace(np.pi / 2 - 0.14, np.pi / 2 - STEP + 0.14, 20)
    ax.plot(R_PLATE * np.cos(th[:-1]), R_PLATE * np.sin(th[:-1]), color=S.INK, lw=1.1, zorder=6)
    ax.add_patch(FancyArrowPatch((R_PLATE * np.cos(th[-2]), R_PLATE * np.sin(th[-2])),
                                 (R_PLATE * np.cos(th[-1]), R_PLATE * np.sin(th[-1])), arrowstyle='-|>',
                                 mutation_scale=8, color=S.INK, lw=1.1, zorder=6))
    ax.annotate('Along the rim', xy=(0.04, 0.262), ha='left', va='bottom', fontsize=8, color=S.INK)
    # Wesley 14 Nov: no 'Next plate', centre cross or 'Payload centre'; 'Radially inward' in the middle of the ring
    ax.add_patch(FancyArrowPatch((sx, sy - 0.0275), (sx, 0.06), arrowstyle='-|>', mutation_scale=8, color=S.INK,
                                 lw=1.1, zorder=6))
    ax.annotate('Radially\ninward', xy=(sx, 0.04), ha='center', va='top', fontsize=8, color=S.INK, linespacing=1.1)
    ax.set_xlim(-0.27, 0.27)
    ax.set_ylim(-0.29, 0.30)
    ax.set_anchor('N')
    ax.axis('off')


def runs_final():
    """7 Nov: the final grid R4034-R4093 (F078/F079), every run welded; the fold-in tilt is plotted for every run,
    including the ten whose landing later aborted (the fold-in completed in each)."""
    d = json.load(open(os.path.join(S.REPO, 'results', 'analysis', 'x11_u_2026-11-07.json')))
    return [{'run': r['run'], 'arm': r['arm'], 'direction': r['direction'], 'd': r['d_cm'],
             'tilt': r['fold_tilt_peak_deg'], 'failures': [], 'verdict': 'VALID'} for r in d['runs']]


def main():
    rows = runs_final()
    S.setup()
    fig = S.figure('full', 2.75)        # 2.5 in of panels plus the legend below
    gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1, 1], wspace=0.16)
    a = fig.add_subplot(gs[0])
    axs = [fig.add_subplot(gs[1]), None]
    axs[1] = fig.add_subplot(gs[2], sharey=axs[0])
    drawing(a)
    arms = (('slot', S.RED, 's', "Attachment point at the plate's centre"), ('measured', S.BLUE, 'o', 'Attachment point at the measured catch point'))
    for ax, direction, xlabel in ((axs[0], 't', 'Offset along the rim (cm)'),
                                  (axs[1], 'ri', 'Offset radially inward (cm)')):
        pass  # 7 Nov: no limit line (Wesley: results as they are, no bars)
        for arm, col, mk, text in arms:
            mine = [r for r in rows if r['arm'] == arm and r['direction'] in ('d0', direction)
                    and r['verdict'] != 'VOID' and r['run'] not in U.VOID_AS_CARD]
            ok = [r for r in mine if not r['failures'] and r['tilt'] is not None]
            ds = sorted({float(r['d']) for r in ok} - {11.6})
            mean = [np.mean([r['tilt'] for r in ok if float(r['d']) == d]) for d in ds]
            ax.plot(ds, mean, color=col, lw=S.LW_HERO, zorder=3)
            ax.plot([float(r['d']) for r in ok], [r['tilt'] for r in ok], mk, ms=S.MS if mk == 'o' else S.MS - 0.4,
                    mfc=col, mec='white', mew=0.4, zorder=4)
        top = 20 if direction == 'ri' else 16
        ax.set_xticks(list(range(0, top + 1, 4)))
        ax.set_xlim(-0.8, top + 0.8)
        ax.set_xlabel(xlabel)
    axs[0].set_ylabel(r'Peak payload tilt ($^\circ$)')
    axs[0].set_ylim(0, 21)
    axs[0].set_yticks([0, 5, 10, 15, 20])
    plt_hide = axs[1].get_yticklabels()
    for lab in plt_hide:
        lab.set_visible(False)
    S.panel(a, 'a', 'Attachment point error')
    S.panel(axs[0], 'b', 'Along the rim')
    S.panel(axs[1], 'c', 'Radially inward')
    from matplotlib.lines import Line2D
    keys = [Line2D([], [], color=col, lw=S.LW_HERO, marker=mk, ms=S.MS if mk == 'o' else S.MS - 0.4, mfc=col,
                   mec='white', mew=0.4) for _, col, mk, _ in arms]
    S.ps.legend_below(fig, keys, [text for *_, text in arms], ncol=2)
    S.save(fig, 'fig_catch_offset_explained' + (f'_{VARIANT}' if VARIANT != 'photo' else ''))


if __name__ == '__main__':
    main()
