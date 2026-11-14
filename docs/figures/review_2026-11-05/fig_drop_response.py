#!/usr/bin/env python3
"""The ring's response to the object landing in the net (Setup 3, 11-13 Nov; feed F113, F116-F118; Wesley 13 Nov:
the drop shows the planner's disturbance rejection and recovery, the drop-off itself is the collaborator's).

The one sentence: when the object landed in the net, the ring's height dipped by about 2 cm and came back within a few
seconds, and its tilt and horizontal error stayed within the motion they already had before the drop, with the ring at
rest or moving.
Hero: three stacked panels against the time from the release (-5 to +5 s, the table's windows), one thin line per drop into the net: (a) the change in
the ring's height from its 5 s pre-release mean, (b) the ring's tilt, (c) its horizontal distance from the reference.
Blue: the ring at rest (measured speed under 0.05 m/s over the second before the release); red: moving (the circle or
the figure-eight). The release is the dashed line at 0.
Left out: the two drops that lost the ring (k1ob_2, k1zo: 57-71 deg, text only), the C++ drop that missed the net
(fm9hc 13 Nov), and every series after the new agent's next descent (fm9_5, fm9c_2, fm9hc_5: q4_next_descent_s).
Data: results/analysis/rig_results_2026-11-13/drop_series.csv.gz (F118: -5 to +10 s on the 50 Hz grid, keyed on flight
and day) and drop_disturbance.csv (in_net, ring_speed_mps, q4_next_descent_s).
"""
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402

S.HERE = HERE
DIR = os.path.join(S.REPO, 'results', 'analysis', 'rig_results_2026-11-13')
REST = 0.05


def main():
    tab = pd.read_csv(os.path.join(DIR, 'drop_disturbance.csv'))
    tab = tab[tab.in_net == 'yes']
    ser = pd.read_csv(os.path.join(DIR, 'drop_series.csv.gz'))
    S.setup()
    fig = S.figure('full', 3.7)
    axs = fig.subplots(3, 1, sharex=True)
    n = {True: 0, False: 0}
    for _, r in tab.iterrows():
        s = ser[(ser.flight == r.flight) & (ser.day == r.day)].sort_values('t_from_release_s')
        t = s.t_from_release_s.to_numpy()
        keep = t <= (r.q4_next_descent_s if np.isfinite(r.q4_next_descent_s) else np.inf)
        rest = r.ring_speed_mps < REST
        n[rest] += 1
        col = S.BLUE if rest else S.RED
        for ax, c in zip(axs, ('ring_dz_mm', 'ring_tilt_deg', 'xy_err_cm')):
            ax.plot(t[keep], s[c].to_numpy()[keep], color=col, lw=S.LW_CTX, alpha=0.85)
    print('drops at rest', n[True], 'moving', n[False])
    for ax in axs:
        ax.axvline(0, **S.REF)
    axs[0].set_ylabel('Height\nchange (mm)')
    axs[1].set_ylabel(r'Payload tilt ($^\circ$)')
    axs[2].set_ylabel('Horizontal\nerror (cm)')
    axs[2].set_xlabel('Time after the release (s)')
    axs[2].set_xlim(-5, 5)                         # the table's windows: 5 s before, 0-5 s after (blind critic 13 Nov)
    axs[0].set_yticks([-25, 0, 25, 50])
    axs[1].set_ylim(0, None)
    axs[2].set_ylim(0, None)
    fig.align_ylabels(axs)
    for ax, letter, name in zip(axs, 'abc', ('Height change', 'Payload tilt', 'Horizontal error')):
        S.panel(ax, letter, name)
    h = [S.plt.Line2D([], [], color=S.BLUE, lw=1.2), S.plt.Line2D([], [], color=S.RED, lw=1.2)]
    S.ps.legend_below(fig, h, ['Payload at rest', 'Payload moving'], ncol=2)
    S.save(fig, 'fig_drop_response')


if __name__ == '__main__':
    main()
