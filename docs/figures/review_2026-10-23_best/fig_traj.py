#!/usr/bin/env python3
"""The figure-8 on the rig with 2, 3 and 4 carriers: the ring's path from above against the path it was asked to fly.

Message: with two, three or four carriers the ring follows the 2 m by 1 m figure-8 within about 2 cm (clock-free RMS
25, 19 and 17 mm), so at print scale its path lies on the reference.
Hero: three plan views at one scale, the reference dashed grey under the ring's path in its fleet size's colour, over
the scored path only (n2_bands' window: the path's first move to the planner's own descent). Each view is drawn about
the path's start, the eight's crossing, where the hold ended (a translation only). The limits follow the data, so a
re-render on other flights (best_data.RIG_BEST) needs no edit here. Left out: the lift, the hold and the landing, the
drones, the error parts (fig_components and fig_summary carry them).
Flights: best_data.RIG_BEST's figure-8 cells (23 Oct: f2slope and f3h, 22 Oct, 0.2 m/s; r2008e60, 7 Oct, the carry
planner at half that speed: not one variable).
"""
import os
import sys

import numpy as np
from matplotlib.ticker import MaxNLocator

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402


def main():
    S.setup()
    fig = S.figure('full', 1.72)
    axs = fig.subplots(1, 3, sharex=True, sharey=True)
    reach = np.zeros(2)
    for ax, n in zip(axs, (2, 3, 4)):
        key = B.RIG_BEST[('figure-8', n)]
        f = B.Flight(key)
        s = B.path_parts(f)
        d = f.run()
        w = s['w']
        ref = d[['payload_ref_x', 'payload_ref_y']].to_numpy()
        P = d[['payload_x', 'payload_y']].to_numpy()
        o = ref[np.argmax(w)]
        ax.plot(*(ref[w] - o).T, **S.REF)
        ax.plot(*(P[w] - o).T, color=S.FLEET_COLOUR[n], lw=1.0, solid_joinstyle='round')
        ax.plot(0, 0, 'o', ms=2.6, color=S.GREY, zorder=4)
        reach = np.maximum(reach, np.nanmax(np.abs(np.vstack([ref[w], P[w]]) - o), axis=0))
        ax.set_aspect('equal')
        S.fleet_panel(ax, chr(ord('a') + n - 2), n)
        ax.annotate(f'{1000 * s["err"]:.0f}' + r'\,mm RMS', xy=(1, 1), xycoords='axes fraction', xytext=(0, 4),
                    textcoords='offset points', ha='right', va='bottom', fontsize=8, color=S.GREY)
        ax.set_xlabel(r'$x$ (m)')
        print(key, f'clock-free {1000 * s["err"]:.1f} mm over {s["window"][1] - s["window"][0]:.1f} s')
    lim = 1.06 * reach
    axs[0].set_xlim(-lim[0], lim[0])
    axs[0].set_ylim(-lim[1], lim[1])
    for axis in (axs[0].xaxis, axs[0].yaxis):
        axis.set_major_locator(MaxNLocator(nbins=3, steps=[1, 2, 2.5, 5, 10], symmetric=True))
    axs[0].set_ylabel(r'$y$ (m)')
    S.g_ticks(*axs)
    S.save(fig, 'fig_traj')


if __name__ == '__main__':
    main()
