#!/usr/bin/env python3
"""The pair's swing in the twin (card docs/experiments/2026-10-23_pair_mode_timing.md, rounds A-D): the 0.55 m pair
hover, the swing's size per timing arm and per fix.

Message: the twin's pair swings only when the throttle is delayed by about 50-60 ms; delay on the rates, the rig's
measured timing and the tip hinge stay calm, and both fixes (the slope filter, the cable pull taken from the model)
remove the 60 ms throttle swing.
Hero: M1(b), the ring normal's tip band-passed 1.1-1.9 Hz, rms from the top to LAND (tools/pair_swing.py, the card's
measure), one marker per run, on a log scale; the card's 1 deg testbed bar dashed. The 60 ms throttle case in red and
its two fixes in blue (round D's comparison); the other arms dark grey.
Runs: A R2449, R2450/R2451, R2452/R2453, R2454/R2455; B R2465/R2470 (rig timing, ema), R2479/R2480 (rig timing + 50 ms
on the throttle); C R2598/R2600 (the tip hinge, planner typed as today); D R2691/R2692 (slope), R2693/R2694 (cable
pull from the model). Left out: B's slope, 1 m, norate and nokff arms and C's re-typed arm (all calm, < 0.1 deg).
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import pair_swing as P  # noqa: E402

ROWS = ((r'60\,ms delay, every channel', ('R2449',), 'ctx'),
        (r'60\,ms delay, throttle', ('R2450', 'R2451'), 'base'),
        (r'$+$ slope filter', ('R2691', 'R2692'), 'fix'),
        (r"$+$ model's cable pull", ('R2693', 'R2694'), 'fix'),
        (r'60\,ms delay, rates', ('R2452', 'R2453'), 'ctx'),
        (r'40\,ms delay, throttle', ('R2454', 'R2455'), 'ctx'),
        (r"rig's timing", ('R2465', 'R2470'), 'ctx'),
        (r"rig's timing $+$ 50\,ms throttle", ('R2479', 'R2480'), 'ctx'),
        (r"tip hinge, rig's timing", ('R2598', 'R2600'), 'ctx'))
TESTBED_DEG = 1.0
GAP_BEFORE = {4, 6}          # a little space after the fixes and before the rig-timing rows


def main():
    S.setup()
    fig = S.figure('full', 2.45)
    ax = fig.subplots()
    ys, y = [], 0.0
    for i, _ in enumerate(ROWS):
        y += 1.0 + (0.45 if i in GAP_BEFORE else 0.0)
        ys.append(y)
    col = {'ctx': S.INK, 'base': S.RED, 'fix': S.BLUE}
    for (label, runs, kind), yy in zip(ROWS, ys):
        vals = [P.score(r)['m1_hover'] for r in runs]
        print(f'{label}: ' + ', '.join(f'{r} {v:.3f}' for r, v in zip(runs, vals)))
        ax.plot(vals, [yy] * len(vals), 'o', ms=S.MS, mfc=col[kind], mec='white', mew=0.5, zorder=3)
        ax.axhline(yy, color=S.FILL, lw=0.6, zorder=0)
    S.ref_line(ax, TESTBED_DEG, horizontal=False)
    ax.annotate(r'swings', xy=(TESTBED_DEG, 1.0), xycoords=('data', 'axes fraction'), xytext=(3, 0),
                textcoords='offset points', ha='left', va='bottom', fontsize=8, color=S.GREY)
    ax.annotate(r'calm', xy=(TESTBED_DEG, 1.0), xycoords=('data', 'axes fraction'), xytext=(-3, 0),
                textcoords='offset points', ha='right', va='bottom', fontsize=8, color=S.GREY)
    ax.set_xscale('log')
    ax.set_xlim(0.005, 20)
    ax.set_xticks([0.01, 0.1, 1, 10])
    ax.set_xticklabels(['0.01', '0.1', '1', '10'])
    ax.minorticks_off()
    ax.set_yticks(ys)
    ax.set_yticklabels([r[0] for r in ROWS])
    for tl, (_, _, kind) in zip(ax.get_yticklabels(), ROWS):
        tl.set_color(col[kind] if kind != 'ctx' else S.INK)
    ax.tick_params(axis='y', length=0)
    ax.spines['left'].set_visible(False)
    ax.set_ylim(ys[-1] + 0.7, ys[0] - 0.7)
    ax.set_xlabel(r'Swing at 1.1--1.9\,Hz, rms ($^\circ$)')
    S.save(fig, 'fig_pair_twin')


if __name__ == '__main__':
    main()
