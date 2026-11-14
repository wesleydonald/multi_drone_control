#!/usr/bin/env python3
"""5 Nov copy of review_2026-11-02/fig_c_time.py: the legend says 'Method of Quan et al.', as the text does (Wesley 5 Nov).

fig:res_compare, redrawn 2 Nov as error over time (Wesley 2 Nov: the u-plot was too messy).

One message: on the 0.4 m/s line the proposed planner follows the reference along the line and stays within a few mm of
it, while the port of Quan et al. lags, stops short of the ends and wanders off the line. (a) the ring's position along
the line against time, with the reference dashed; (b) the ring's distance from the line against time. Time from the
path's start. Runs: the planner R2737 (the table's run), the port R2689 and R2690 (search's best gains). Data and
windows exactly as review_2026-10-23/fig_c_line.py (tools/c_metrics.py scoring), plotted against time instead.
"""
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'review_2026-10-23'))
import review_style as S  # noqa: E402
import c_metrics as C  # noqa: E402
import metrics as M  # noqa: E402

OURS, NET = ('R4112',), ('R4124', 'R4125')     # 7 Nov: the final comparison (F082/F083); were R2737, R2689-R2690


def line_run(rid):
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
    return t[win] - t_p0, act[win, 0] - x0, ref[win, 0] - off[0] - x0, cf[win], dist


def main():
    S.setup()
    fig = S.figure('full', 3.0)
    axs = fig.subplots(2, 1, sharex=True)
    tmax = 0
    for rids, col in ((NET, S.RED), (OURS, S.BLUE)):
        for k, rid in enumerate(rids):
            tt, u, uref, e, dist = line_run(rid)
            if rid == OURS[0]:
                axs[0].plot(tt, uref, color='0.45', lw=0.9, ls='--', zorder=1)
            axs[0].plot(tt, u, color=col, lw=1.0, alpha=0.9, zorder=3)
            axs[1].plot(tt, 1000 * e, color=col, lw=1.0, alpha=0.9, zorder=3)
            tmax = max(tmax, tt.max())
            print(rid, f'{tt.min():.1f}..{tt.max():.1f} s, cf rms {1000*np.sqrt(np.mean(e**2)):.1f} mm')
    axs[0].set_ylabel('Position along\nthe line (m)')
    axs[0].set_yticks([0, 0.5, 1])
    axs[0].set_yticklabels(['0', '0.5', '1'])
    axs[1].set_ylabel('Distance from\nthe line (mm)')
    axs[1].set_xlabel('Time from the start of the path (s)')
    axs[1].set_xlim(0, min(tmax, 60))
    axs[1].set_ylim(0, 30)
    axs[0].set_ylim(-0.1, dist + 0.15)
    S.panel(axs[0], 'a', 'Position along the line')
    S.panel(axs[1], 'b', 'Distance from the line')
    import matplotlib.lines as ml
    h = [ml.Line2D([], [], color=S.BLUE, lw=1.2), ml.Line2D([], [], color=S.RED, lw=1.2),
         ml.Line2D([], [], color='0.45', lw=0.9, ls='--')]
    S.ps.legend_below(fig, h, ["Proposed planner", 'Method of Quan et al.', 'Reference'], ncol=3)
    fig.savefig(os.path.join(HERE, 'fig_c_time.pdf'))
    fig.savefig(os.path.join(HERE, 'fig_c_time.png'), dpi=200)


if __name__ == '__main__':
    main()
