#!/usr/bin/env python3
"""C: our planner against our port of Quan et al.'s network on the 0.4 m/s line (card
docs/experiments/2026-10-23_paper_method_compare.md, Results).

Message: on the 0.4 m/s line our planner keeps the ring within 3 mm of the line, the network lets it wander up to
28 mm and turns further short of the ends: the clock-free path error, drawn without the clock.
Hero: the ring's distance from the line against its position along the line, every pass of the scored window (8 cycles),
our planner in blue (R2448), the network at the search's best gains in red (R2689, R2690). The line's ends are the
dashed verticals. Left out: the lag (reported apart, as Wesley scores it), tilt, yaw (fig_c_yaw), the hover and the
0.2 m/s cells. Windows and the target exactly as tools/c_metrics.py scores them (from PATH_START over the whole
cycles before LAND, against the target from before the hand-over).
Preliminary: our planner's line 0.4 has one run (its 2nd flies with the C cells still owed).
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import c_metrics as C  # noqa: E402
import metrics as M  # noqa: E402

OURS, NET = ('R2448',), ('R2689', 'R2690')


def line_run(rid):
    """(u along the line in m, distance from the line in m) over the scored window, and the line's length."""
    d = C.run_dirs([rid])[0]
    s = C.score(d)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv')).drop_duplicates('t').reset_index(drop=True)
    p = C.planner_params(d)
    t = run.t.to_numpy()
    ref = run[['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy()
    act = run[['payload_x', 'payload_y', 'payload_z']].to_numpy()
    ev = M.read_event_list(d)
    t_p0 = C._first(ev, 'PATH_START')
    t_at = min(s['t_handover'], t_p0)
    sel = (t >= t_at - 1.0) & (t < t_at)
    tgt0 = np.nanmedian(ref[sel][-5:], axis=0)
    own0 = np.array([float(np.interp(t_p0, t, ref[:, c])) for c in range(3)])
    off = own0 - tgt0
    x0, dist = own0[0] - off[0], float(p.get('traj_distance', 1.0))
    cf = M.segment_error(act, x0, x0 + dist, ref[:, 1] - off[1], ref[:, 2] - off[2], past_end=False)
    w0, w1 = s['window']
    win = (t >= w0) & (t <= w1)
    return act[win, 0] - x0, cf[win], dist, s


def main():
    S.setup()
    fig = S.figure('full', 2.85)        # 2.6 in of panels plus the legend below
    axs = fig.subplots(2, 1, sharex=True, sharey=True)
    dist = None
    for ax, rids, col, text in ((axs[0], OURS, S.BLUE, 'our planner'),
                                (axs[1], NET, S.RED, 'network')):
        for k, rid in enumerate(rids):
            u, e, dist, s = line_run(rid)
            ax.plot(u, 1000 * e, color=col, lw=S.LW_CTX, alpha=0.9, label=text if k == 0 else None)
            print(rid, s['arm'], f"cf {s['cf_rms_mm']:.1f} rms / {s['cf_max_mm']:.1f} max mm, "
                  f"reach {s['reach_mean_mm']:.1f} mm, u {u.min():.3f}..{u.max():.3f}")
        for x in (0.0, dist):
            S.ref_line(ax, x, horizontal=False)
    ends, = axs[0].plot([], [], **S.REF)
    axs[1].set_xlabel(r'Ring position along the line (m)')
    axs[1].set_xlim(-0.05, dist + 0.05)
    axs[1].set_ylim(0, 30)
    axs[1].set_yticks([0, 10, 20, 30])
    axs[1].set_xticks([0, 0.3, 0.6, 0.9])
    fig.supylabel(r'Distance from the line (mm)', fontsize=9)
    S.ps.panel_label(axs[0], 'a')
    S.ps.panel_label(axs[1], 'b')
    (h_ours,), (h_net,) = (ax.get_legend_handles_labels()[0] for ax in axs)
    S.ps.legend_below(fig, [h_ours, h_net, ends], ['our planner', 'network', 'ends of the line'], ncol=3)
    S.save(fig, 'fig_c_line')


if __name__ == '__main__':
    main()
