#!/usr/bin/env python3
"""The ring's heading on the rig's spinny line (three carriers, 22 Oct): the one motion that also asks for a yaw.

Message: through the full 360 deg turn at a constant rate the ring's heading runs about 0.5 s behind its reference:
the yaw error holds near -12 deg while the rate is constant (mean -9.4, RMS 10.5, at most 16 deg), and is about 2 deg RMS
once that delay is taken out.
Hero: (a) the ring's yaw (its fleet colour) and the yaw it was asked for (dashed grey), from the path's start to its
end; (b) the error, ring minus reference. Left out: the cable azimuths, the drones, the hold before the path.
Flight: best_data.RIG_BEST's spinny line (Wesley 23 Oct: ls3c, today's constant-rate full turn; ls3, flown with the
older law of one radian per 0.5 m, had the lower error: in the index). Numbers and series: tools/yaw_track.track
(read-only; the old law only for the keys in best_data.YAW_OLD_LAW).
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402
import rig_summary  # noqa: E402
import yaw_track  # noqa: E402

KEY = B.RIG_BEST[('spinny line', 3)]


def main():
    f = B.Flight(KEY)
    rig_summary.resolve = lambda arg: (f.run_dir, f.logs, None, f.name, '')   # read the run as it is: no conversion
    r = yaw_track.track(KEY, old_law=KEY in B.YAW_OLD_LAW)
    x, yaw, des, err, rate, T = r['series']
    print(f"{KEY}: error mean {r['err_mean']:+.1f} rms {r['err_rms']:.1f} max {r['err_max']:.1f} deg; delay "
          f"{r['delay_s']:+.2f} s (rms {r['rms_after_delay']:.1f} after); turned {r['turned']:.0f} of "
          f"{r['turn_des']:.0f} deg")
    c = S.FLEET_COLOUR[f.n]
    m = (x >= 0) & (x <= T)
    y0 = des[m][0]
    S.setup()
    fig = S.figure('full', 2.7)
    axs = fig.subplots(2, 1, sharex=True, gridspec_kw=dict(height_ratios=[1.6, 1]))
    axs[0].plot(x[m], des[m] - y0, **S.REF)
    axs[0].plot(x[m], yaw[m] - y0, color=c, lw=S.LW_HERO)
    k = int(np.argmin(np.abs(x - 0.55 * T)))
    axs[0].annotate('reference', xy=(x[k], des[k] - y0), xytext=(-4, 4), textcoords='offset points', ha='right',
                    va='bottom', fontsize=8, color=S.GREY)
    axs[0].annotate('ring', xy=(x[k], yaw[k] - y0), xytext=(4, -4), textcoords='offset points', ha='left', va='top',
                    fontsize=8, color=c)
    turn = 90 * math.ceil(max(np.nanmax(des[m] - y0), np.nanmax(yaw[m] - y0)) / 90)
    axs[0].set_ylabel(r'Yaw ($^\circ$)')
    axs[0].set_ylim(-0.08 * turn, 1.04 * turn)
    axs[0].set_yticks(range(0, turn + 1, 90))
    axs[1].plot(x[m], err[m], color=c, lw=S.LW_CTX)
    axs[1].axhline(0, color=S.LIGHT, lw=0.5, zorder=0)
    lo = -5 * math.ceil(-np.nanmin(err[m]) / 5 + 0.05)
    axs[1].set_ylim(lo, 5)
    axs[1].set_yticks(range(lo + (-lo) % 10, 1, 10) if lo <= -10 else [lo, 0])
    axs[1].set_ylabel(r'Error ($^\circ$)')
    axs[1].set_xlim(0, T)
    axs[1].set_xticks(range(0, int(T) + 1, 5))
    axs[1].set_xlabel(r'$t$ from the path start (s)')
    fig.align_ylabels(axs)
    S.ps.panel_label(axs[0], 'a')
    S.ps.panel_label(axs[1], 'b')
    S.save(fig, 'fig_yaw')


if __name__ == '__main__':
    main()
