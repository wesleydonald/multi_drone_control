#!/usr/bin/env python3
"""U: the magnet caught off its slot (card docs/experiments/2026-10-23_catch_offset.md, Results).

Message: radially in, folding in at the measured catch point keeps the ring's fold-in tilt near 1-2 deg to 16 cm,
while assuming the slot lets it grow past the 5 deg bar from 8 cm; along the rim the order reverses past 8 cm.
Hero: the fold-in tilt peak against the catch offset, the two arms in blue (measured) and red (slot assumed), one
marker per run, the line through the cell means. Left out: the separation (its own finding), the steady tilt, the
path after, the VOID runs (re-flown; preliminary). Data: R2629-R2684 through tools/catch_offset.analyse.
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import catch_offset as C  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402

RUNS = 'R2629-R2684'
# the card's Results class R2634 as VOID: its only failure is the runner's mocap-staleness race 2 s after LANDED,
# outside every scored window (the scorer's code counts any manifest failure); it is re-flown
VOID_AS_CARD = {'R2634'}
CACHE = os.path.join(S.SCRATCH, 'u_runs.json')


def runs():
    if os.path.exists(CACHE):
        return json.load(open(CACHE))
    out = []
    for d in C.run_dirs([RUNS]):
        r = C.analyse(d)
        out.append(dict(run=r['run'], arm=r['arm'], direction=r['direction'], d=r['d_cm'], verdict=C.verdict(r),
                        failures=r['failures'], void=r['void'], tilt=r.get('fold_tilt_peak_deg')))
    os.makedirs(S.SCRATCH, exist_ok=True)
    json.dump(out, open(CACHE, 'w'), indent=1)
    return out


def main():
    rows = runs()
    S.setup()
    fig = S.figure('full', 2.35)
    axs = fig.subplots(1, 2, sharey=True)
    arms = (('slot', S.RED, 's', 'slot assumed'), ('measured', S.BLUE, 'o', 'measured catch point'))
    for ax, direction, xlabel in ((axs[0], 't', r'Catch offset along the rim (cm)'),
                                  (axs[1], 'ri', r'Catch offset radially inward (cm)')):
        S.ref_line(ax, C.FOLD_TILT_BAR_DEG, r'$5^\circ$ bar' if direction == 'ri' else None)
        for arm, col, mk, text in arms:
            mine = [r for r in rows if r['arm'] == arm and r['direction'] in ('d0', direction)
                    and r['verdict'] != 'VOID' and r['run'] not in VOID_AS_CARD]
            ok = [r for r in mine if not r['failures'] and r['tilt'] is not None]
            # 11.6 cm is the neighbouring plate's centre (a wrong-plate catch): drawn, but not on the grid's line
            ds = sorted({float(r['d']) for r in ok} - {11.6})
            mean = [np.mean([r['tilt'] for r in ok if float(r['d']) == d]) for d in ds]
            ax.plot(ds, mean, color=col, lw=S.LW_HERO, zorder=3)
            ax.plot([float(r['d']) for r in ok], [r['tilt'] for r in ok], mk, ms=S.MS if mk == 'o' else S.MS - 0.4,
                    mfc=col, mec='white', mew=0.4, zorder=4)
            for r in mine:
                if not r['failures']:
                    continue
                y = r['tilt'] if r['tilt'] is not None else 0.0
                ax.plot(float(r['d']), y, 'x', ms=4, mew=0.9, color=col, zorder=4, clip_on=False)
                ax.annotate('no weld' if r['tilt'] is None else 'late weld', xy=(float(r['d']), y),
                            xytext=(0, 4) if r['tilt'] is None else (2, -3), textcoords='offset points',
                            ha='center' if r['tilt'] is None else 'right', va='bottom' if r['tilt'] is None
                            else 'top', fontsize=7, color=col)
            if direction == 'ri':
                at = 12.0 if arm == 'slot' else 10.0
                y = float(np.interp(at, ds, mean))
                ax.annotate(text, xy=(at, y), xytext=(-6, 4) if arm == 'slot' else (0, 5),
                            textcoords='offset points', ha='right' if arm == 'slot' else 'center',
                            va='bottom', fontsize=8, color=col)
        ax.set_xlabel(xlabel)
        ax.set_xticks([0, 4, 8, 12, 16])
        ax.set_xlim(-0.8, 16.8)
    axs[0].set_ylabel(r'Fold-in ring tilt peak ($^\circ$)')
    axs[0].set_ylim(0, 18)
    axs[0].set_yticks([0, 5, 10, 15])
    S.ps.panel_label(axs[0], 'a')
    S.ps.panel_label(axs[1], 'b')
    S.save(fig, 'fig_u_catch_offset')


if __name__ == '__main__':
    main()
