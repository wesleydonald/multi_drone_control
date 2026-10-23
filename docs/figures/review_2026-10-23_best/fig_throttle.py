#!/usr/bin/env python3
"""Each carrier's throttle against its cap on the rig's figure-8, with 2, 3 and 4 carriers.

Message: no carrier reached its cap. A carrier of four flew at about 0.62 (cap 0.8), of three at about 0.66 (cap 0.8);
the pair flew at 0.72-0.75 with its cap raised to 0.9, so the pair has the least margin: the default 0.8 cap would
have left 0.01-0.05.
Two designs (Wesley picks):
  fig_throttle_box   one row per carrier, grouped by fleet size, in its colour: the throttle's spread (box: quartiles,
                     whiskers 5-95 %, the peak as a dot) against the cap (grey dashed tick).
  fig_throttle       (a-c) one panel per fleet size: each carrier's throttle in time, in its fleet colour, and the cap.
Throttle: the tracker's thr_out, the command sent to the flight controller (the thrust offset and the battery term
included), over the scored path (n2_bands' window). Cap: the tracker's throttle_max (params.json).
"""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402


def throttle(n):
    f = B.Flight(B.RIG_BEST[('figure-8', n)])
    s = B.path_parts(f)
    w0, w1 = s['window']
    cap = json.load(open(glob.glob(os.path.join(f.logs, 'logs', 'tracker', 'drone0_*', 'params.json'))[0]))['throttle_max']
    out = []
    for i, d in f.trackers().items():
        m = (d.t >= w0) & (d.t <= w1)
        out.append((d.t[m].to_numpy() - w0, d.thr_out[m].to_numpy()))
    return out, float(cap)


def main():
    data = {n: throttle(n) for n in (2, 3, 4)}
    for n, (tr, cap) in data.items():
        print(n, 'cap', cap, ' '.join(f'd{i}: med {np.median(x):.3f} max {np.max(x):.3f}' for i, (_, x) in enumerate(tr)))

    S.setup()
    fig = S.figure('full', 3.1)
    axs = fig.subplots(3, 1, sharey=True)
    for ax, n in zip(axs, (2, 3, 4)):
        tr, cap = data[n]
        for t, x in tr:
            ax.plot(t, x, color=S.FLEET_COLOUR[n], lw=S.LW_CTX)
        S.ref_line(ax, cap, f'cap {cap:.1f}', x=1.0, ha='right', va='bottom')
        ax.set_xlim(0, max(t[-1] for t, _ in tr))
        S.fleet_panel(ax, chr(ord('a') + n - 2), n)
    axs[0].set_ylim(0.5, 1.0)
    axs[0].set_yticks([0.5, 0.75, 1.0])
    axs[1].set_ylabel('Throttle')
    axs[2].set_xlabel(r'$t$ from the path start (s)')
    fig.align_ylabels(axs)
    S.g_ticks(*axs, which='y')
    S.save(fig, 'fig_throttle')

    from matplotlib.patches import Rectangle
    S.setup()
    fig = S.figure('full', 2.3)
    ax = fig.subplots()
    y, ticks, labels = 0.0, [], []
    for n in (2, 3, 4):
        tr, cap = data[n]
        c = S.FLEET_COLOUR[n]
        ys = []
        for i, (_, x) in enumerate(tr):
            p5, q1, med, q3, p95 = np.percentile(x, [5, 25, 50, 75, 95])
            ax.plot([p5, q1], [y, y], color=c, lw=S.LW_CTX)
            ax.plot([q3, p95], [y, y], color=c, lw=S.LW_CTX)
            ax.add_patch(Rectangle((q1, y - 0.3), q3 - q1, 0.6, fc=S.tint(c, 0.3), ec=c, lw=S.LW_CTX))
            ax.plot([med, med], [y - 0.3, y + 0.3], color=c, lw=S.LW_HERO)
            ax.plot(np.max(x), y, 'o', ms=2.4, color=c)
            ys.append(y)
            y -= 1.0
        ax.plot([cap, cap], [ys[0] + 0.5, ys[-1] - 0.5], **S.REF)
        ticks.append(np.mean(ys))
        labels.append(f'{n} carriers')
        y -= 0.8
    ax.annotate('cap', xy=(data[2][1], 0.5), xytext=(0, 2), textcoords='offset points', ha='center', va='bottom',
                fontsize=8, color=S.GREY)
    ax.set_yticks(ticks, labels)
    for t, n in zip(ax.get_yticklabels(), (2, 3, 4)):
        t.set_color(S.FLEET_COLOUR[n])
    ax.tick_params(axis='y', length=0)
    ax.set_xlim(0.5, 1.0)
    ax.set_xticks([0.5, 0.6, 0.7, 0.8, 0.9, 1.0])
    ax.set_xlabel('Throttle on the figure-8')
    S.g_ticks(ax, which='x')
    S.save(fig, 'fig_throttle_box')


if __name__ == '__main__':
    main()
