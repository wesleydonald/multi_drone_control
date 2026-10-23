#!/usr/bin/env python3
"""The pair's swing on the rig, 22 Oct (cards docs/experiments/2026-10-22_velocity_filter.md and
2026-10-23_pair_mode_timing.md): two quads on 0.55 m cables, the carriers' mocap velocity filter ema against slope.

Message: with the rig's ema filter the pair's 1.4 Hz see-saw grows after lift-off until the flight is cut (the loss
detector, or LAND by hand); with the slope filter it stays about half as large and decays.
Hero: the ring's see-saw (its tilt about the axis across the pair's line: tools/pair_swing.py's see-saw, unfiltered)
from lift-off, like for like (Wesley 23 Oct): ema h2t9p red against slope h2slope blue (same plates and settings, only
the filter differs; h2, h2rts and the figure-8 f2slope are in the text), each drawn to its window's
end (tools/pair_swing.windows: LAND, the detection, or the planner's own descent), cut at 9 s. h2t9p's end is the
operator's LAND (open circle; registry row RIG-1022-h2t9p). Left out: the roll about
the pair's line (slower, 0.6-0.8 Hz), the band-pass M1 numbers (in the card), the drones.
Not one variable: h2 flew the pair on plates 0/180 at the 0.8 throttle cap; the other four on 330/150 at 0.9.
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import pair_swing as P  # noqa: E402
import rig_summary  # noqa: E402

EMA, SLOPE = ('h2t9p',), ('h2slope',)   # like for like (Wesley 23 Oct): same plates and settings, only the filter
CUT_S = 9.0


def seesaw(arg):
    run, _, _, label, _ = rig_summary.resolve(arg)
    d = pd.read_csv(os.path.join(run, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    ev = pd.read_csv(os.path.join(run, 'logs', 'events.csv'))
    a0, _, e, why = P.windows(d, ev)
    w, x, y, z = d[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy().T
    nx, ny = 2 * (x * z - w * y), 2 * (y * z + w * x)
    az = np.unwrap(np.arctan2(d.d0_y - d.d1_y, d.d0_x - d.d1_x).to_numpy())
    k = 25
    az = np.convolve(np.pad(az, k, mode='edge'), np.ones(2 * k + 1) / (2 * k + 1), mode='valid')
    see = np.degrees(np.arcsin(np.clip(-(nx * np.cos(az) + ny * np.sin(az)), -1, 1)))
    t = d.t.to_numpy()
    sel = slice(a0, e)
    return t[sel] - t[a0], see[sel], why


def main():
    S.setup()
    fig = S.figure('full', 2.6)
    axs = fig.subplots(2, 1, sharex=True, sharey=True)
    lim = 12.0
    for ax, runs, col, text in ((axs[0], EMA, S.RED, 'ema filter'), (axs[1], SLOPE, S.BLUE, 'slope filter')):
        for arg in runs:
            tt, see, why = seesaw(arg)
            keep = tt <= CUT_S
            ax.plot(tt[keep], see[keep], color=col, lw=S.LW_CTX)
            print(arg, f'{tt[-1]:.1f} s from lift-off, ended by {why}; see-saw p-p over the first 6 s '
                  f'{np.ptp(see[tt <= 6.0]):.1f} deg')
            if tt[-1] < CUT_S:
                # how the flight ended: the drop rule (x) or the operator's LAND (open circle)
                y = float(np.clip(see[-1], -lim, lim))
                dropped = why == 'DETECTED'
                ax.plot(tt[-1], y, 'x' if dropped else 'o', color=col, mfc='white', ms=4 if dropped else 3.4,
                        mew=0.9, clip_on=False, zorder=4)
                ax.annotate('dropped' if dropped else 'operator LAND', xy=(tt[-1], y), xytext=(6, -1 if dropped else 0),
                            textcoords='offset points', ha='left', va='top' if dropped else 'center', fontsize=8,
                            color=col) if (dropped or tt[-1] > 6.0) else None
        ax.axhline(0, color=S.LIGHT, lw=0.5, zorder=0)
        ax.annotate(text, xy=(1.0, 1.0), xycoords='axes fraction', xytext=(-2, -1), textcoords='offset points',
                    ha='right', va='top', fontsize=8, color=col)
    axs[1].set_xlim(0, CUT_S)
    axs[1].set_ylim(-lim, lim)
    axs[1].set_yticks([-10, 0, 10])
    axs[1].set_xlabel(r'$t$ after lift-off (s)')
    fig.supylabel(r'Ring see-saw ($^\circ$)', fontsize=9)
    S.ps.panel_label(axs[0], 'a')
    S.ps.panel_label(axs[1], 'b')
    S.save(fig, 'fig_pair_rig')


if __name__ == '__main__':
    main()
