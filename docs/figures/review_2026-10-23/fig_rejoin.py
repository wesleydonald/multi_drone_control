#!/usr/bin/env python3
"""The rejoin of a departed carrier (card docs/experiments/2026-10-23_rejoin_departed.md, E2; run R2594).

Message: a carrier released from the T flies back to its own plate (climb, transit over the plate, descent), is
re-welded there and folded back in, and the ring's tilt stays under 1.5 deg throughout (bar 5 deg).
Hero (a): the rejoiner's body path seen from the side (its plane: x = 0 in this run), from the release to LAND, with
its rod at the weld and folded in (pivot 4 cm under the body, straight to the plate). Support (b): the ring's tilt from
the release to LAND, the staged hold, the approach and the fold-in blend shaded, the 5 deg fold-in bar dashed.
Left out: the opposite pair (it flies on, ~0.74 m either side along x), the rod while free (no tip log in this run),
the lift and the landing. Events from events.csv; positions and tilt from run.csv as recorded.
"""
import csv
import glob
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402

RUN, DRONE = 'R2594', 1
PIVOT_DZ = 0.04          # the pivot sits 4 cm under the body (rig geometry, launch arg pivot_offset_z -0.04)
TILT_BAR_DEG = 5.0
DATA = '#6a00a8'          # the 4-carrier colour of the fleet-size palette       # the card's fold-in bar (weld .. +15 s)


def main():
    d = glob.glob(os.path.join(S.REPO, 'results', '*', f'{RUN}_*'))[0]
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t')
    ev = {}
    for e in csv.DictReader(open(os.path.join(d, 'logs', 'events.csv'))):
        ev.setdefault(e['event'], float(e['sim_time']))
    t = run.t.to_numpy()
    body = run[[f'd{DRONE}_x', f'd{DRONE}_y', f'd{DRONE}_z']].to_numpy()
    ring = run[['payload_x', 'payload_y', 'payload_z']].to_numpy()
    t0, t1 = ev['RELEASE'], ev['LAND']
    w = (t >= t0) & (t <= t1)
    tb0 = ev['REJOIN_START'] - 10.0          # (b) opens 10 s before the rejoin command, in the staged hover
    wb = (t >= tb0) & (t <= t1)

    def at(tt):
        return int(np.searchsorted(t, tt))

    S.setup()
    fig = S.figure('full', 2.55)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.55])
    ax = fig.add_subplot(gs[0])
    ax_t = fig.add_subplot(gs[1])

    # (a) side view in the rejoiner's plane (y, z); the plate is the rim point on the rejoiner's side
    i_w = at(ev['REJOIN_WELD'])
    rc = ring[i_w]
    plate = np.array([rc[1] - 0.225, rc[2] + 0.02])
    r_lo, r_hi = rc[1] - 0.225, rc[1] + 0.225
    ax.plot([r_lo, r_hi], [rc[2], rc[2]], color=S.INK, lw=2.2, solid_capstyle='butt', zorder=3)
    ax.annotate('ring', xy=(0.5 * (plate[0] + r_hi), rc[2]), xytext=(0, -5), textcoords='offset points', ha='center',
                va='top', fontsize=8, color=S.GREY)     # under the bar: its end now meets the box's right side
    for tt, name in ((ev['REJOIN_WELD'], 'weld'), (ev['REJOIN_BLEND_OVER'] + 2.0, 'folded in')):
        i = at(tt)
        piv = np.array([body[i, 1], body[i, 2] - PIVOT_DZ])
        ax.plot([piv[0], plate[0]], [piv[1], plate[1]], color=S.GREY, lw=S.LW_CTX, zorder=2)
    out = (t >= t0) & (t <= ev['REJOIN_START'])
    back = (t >= ev['REJOIN_START']) & (t <= t1)
    ax.plot(body[out, 1], body[out, 2], color=S.GREY, lw=S.LW_CTX, ls=(0, (1.5, 1.5)), zorder=4)
    ax.plot(body[back, 1], body[back, 2], color=DATA, lw=S.LW_HERO, zorder=4)
    # arrowheads along the return: the climb, the transit over the plate, the descent, the fold-in
    for frac in (0.08, 0.3, 0.62, 0.86):
        tt = ev['REJOIN_START'] + frac * (ev['REJOIN_BLEND_OVER'] - ev['REJOIN_START'])
        if ev['REJOIN_WELD'] - 4.0 < tt < ev['FOLD_IN'] + 1.0:
            continue
        i, j = at(tt), at(tt + 0.4)
        ax.annotate('', xy=(body[j, 1], body[j, 2]), xytext=(body[i, 1], body[i, 2]),
                    arrowprops=dict(arrowstyle='-|>', color=DATA, lw=0, mutation_scale=7), zorder=6)
    marks = (('RELEASE', 'carrying', (7, -3), 'left'), ('REJOIN_START', 'staged', (0, -5), 'center'),
             ('REJOIN_WELD', 'weld', (6, 0), 'left'))
    for key, text, off, ha in marks:
        i = at(ev[key])
        ax.plot(body[i, 1], body[i, 2], 'o', ms=3.6, mfc='white', mec=DATA, mew=0.9, zorder=7)
        ax.annotate(text, xy=(body[i, 1], body[i, 2]), xytext=off, textcoords='offset points', ha=ha,
                    va={'carrying': 'top', 'staged': 'top'}.get(text, 'center'), fontsize=8, color=S.INK)
    ax.plot(plate[0], plate[1], 'o', ms=3, color=S.INK, zorder=5)
    ax.annotate('plate', xy=plate, xytext=(-5, -6), textcoords='offset points', ha='right', va='top', fontsize=8,
                color=S.GREY)
    ax.set_aspect('equal')
    ax.set_xlim(-1.2, 0.45)
    ax.set_ylim(0.8, 2.45)
    ax.set_xticks([-1.0, -0.5, 0.0])
    ax.set_yticks([1.0, 1.5, 2.0])
    ax.set_xlabel(r'$y$ (m)')
    ax.set_ylabel(r'$z$ (m)')

    # (b) the ring's tilt, release to LAND
    tilt = run.payload_tilt_deg.to_numpy()
    ta = ev['REJOIN_START']
    for a, b, text in ((ev['REJOIN_START'], ev['REJOIN_WELD'], 'approach'),
                       (ev['FOLD_IN'], ev['REJOIN_BLEND_OVER'], 'fold-in')):
        ax_t.axvspan(a - ta, b - ta, color=S.FILL, lw=0, zorder=0)
        ax_t.annotate(text, xy=((a + b) / 2 - ta, 1.0), xycoords=('data', 'axes fraction'), xytext=(0, -2),
                      textcoords='offset points', ha='center', va='top', fontsize=8, color=S.GREY)
    S.ref_line(ax_t, TILT_BAR_DEG, r'$5^\circ$ bar')
    ax_t.plot(t[wb] - ta, tilt[wb], color=DATA, lw=S.LW_CTX, rasterized=True)
    win = (t >= ev['REJOIN_WELD']) & (t <= ev['REJOIN_WELD'] + 15.0)
    print(f"weld {ev['REJOIN_WELD'] - ev['REJOIN_START']:.2f} s after the ATTACH; fold-in tilt peak "
          f"{np.nanmax(tilt[win]):.2f} deg; panel (b) peak {np.nanmax(tilt[wb]):.2f} deg")
    ax_t.set_xlim(tb0 - ta, t1 - ta)
    ax_t.set_ylim(0, 6)
    ax_t.set_yticks([0, 2, 4, 6])
    ax_t.set_xlabel(r'$t$ after the rejoin command (s)')
    ax_t.set_ylabel(r'Ring tilt ($^\circ$)')
    S.ps.panel_label(ax, 'a')
    S.ps.panel_label(ax_t, 'b')
    S.save(fig, 'fig_rejoin')


if __name__ == '__main__':
    main()
