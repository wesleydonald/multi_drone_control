#!/usr/bin/env python3
"""The planner's cable tensions against the static share and against the pull the rig measures, 2, 3 and 4 carriers.

Message: the planner's tensions sit on the static share of the ring's weight (m g / n sin 60 deg: 4.9, 3.2 and 2.4 N)
within 3 %; the pull the trackers measure from motion capture reads 13-23 % above it.
Two designs (Wesley picks):
  fig_tension          one panel: carriers on x; the static share (grey dashed), each cable's planned tension (blue) and
                       each carrier's measured pull (red), means over the figure-8, the pull's 5-95 % as a light red bar.
  fig_tension_eight    three carriers along the figure-8: each cable's planned tension (blue) and each carrier's
                       measured pull (light red) in time, the static share grey dashed.
Planned: the planner log's t<slot> (node 0 of the published plan). Measured pull: the drone's mass times the cable
acceleration the tracker measured (tracker log acx/acy/acz: the mocap acceleration less the modelled thrust and gravity,
the input its cable_source=mocap model used), as recorded. Static share: the ring's 0.86 kg over n cables at the planned
60 deg elevation (planner params load_mass, cable_elev_deg). Window: the scored path (n2_bands).
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402


def tensions(n):
    f = B.Flight(B.RIG_BEST[('figure-8', n)])
    s = B.path_parts(f)
    w0, w1 = s['window']
    p = f.planner()
    mp = (p.t >= w0) & (p.t <= w1)
    planned = [(p.t[mp].to_numpy() - w0, p[f't{i}'][mp].to_numpy()) for i in range(n)]
    md = float(f.params['drone_mass'])
    pull = []
    for i, d in f.trackers().items():
        m = (d.t >= w0) & (d.t <= w1)
        pull.append((d.t[m].to_numpy() - w0, md * np.sqrt(d.acx[m] ** 2 + d.acy[m] ** 2 + d.acz[m] ** 2).to_numpy()))
    share = float(f.params['load_mass']) * B.G / n / np.sin(np.radians(float(f.params['cable_elev_deg'])))
    return planned, pull, share


def main():
    data = {n: tensions(n) for n in (2, 3, 4)}
    for n, (pl, pu, sh) in data.items():
        pm = [np.nanmean(x) for _, x in pl]
        um = [np.nanmean(x) for _, x in pu]
        print(n, f'share {sh:.2f} N; planned ' + ' '.join(f'{v:.2f}' for v in pm)
              + f' ({100 * (np.mean(pm) / sh - 1):+.1f} %); pull ' + ' '.join(f'{v:.2f}' for v in um)
              + f' ({100 * (np.mean(um) / sh - 1):+.1f} %)')

    S.setup()
    fig = S.figure('half', 2.3)
    ax = fig.subplots()
    nn = np.array([2, 3, 4])
    for n in nn:
        ax.plot([n - 0.36, n + 0.36], [data[n][2]] * 2, **S.REF)
    for n in nn:
        pl, pu, sh = data[n]
        k = len(pl)
        off = (np.arange(k) - (k - 1) / 2) * 0.07
        for o, (_, x) in zip(off, pl):
            ax.plot(n - 0.16 + o, np.nanmean(x), 'o', ms=S.MS - 0.6, color=S.BLUE)
        for o, (_, x) in zip(off, pu):
            lo, hi = np.nanpercentile(x, [5, 95])
            ax.plot([n + 0.16 + o] * 2, [lo, hi], color=S.RED_LIGHT, lw=S.LW_CTX, zorder=1)
            ax.plot(n + 0.16 + o, np.nanmean(x), 'o', ms=S.MS - 0.6, color=S.RED, zorder=2)
    ax.annotate('static share', xy=(2 - 0.36, data[2][2]), xytext=(0, -3), textcoords='offset points',
                ha='left', va='top', fontsize=8, color=S.GREY)
    ax.annotate('planned', xy=(4 - 0.16, data[4][2]), xytext=(0, -10), textcoords='offset points', ha='center',
                va='top', fontsize=8, color=S.BLUE)
    ax.annotate('measured', xy=(4 + 0.16, np.nanmean(data[4][1][1][1])), xytext=(0, 12), textcoords='offset points',
                ha='center', va='bottom', fontsize=8, color=S.RED)
    ax.set_xticks([2, 3, 4])
    for t, n in zip(ax.get_xticklabels(), (2, 3, 4)):
        t.set_color(S.FLEET_COLOUR[n])
    ax.set_xlim(1.6, 4.5)
    ax.set_ylim(0, 6.5)
    ax.set_yticks([0, 2, 4, 6])
    ax.set_xlabel('Carriers')
    ax.set_ylabel('Cable tension (N)')
    S.save(fig, 'fig_tension', width='half')

    S.setup()
    fig = S.figure('full', 2.0)
    ax = fig.subplots()
    pl, pu, sh = data[3]
    for t, x in pu:
        ax.plot(t, x, color=S.RED_LIGHT, lw=S.LW_CTX)
    for t, x in pl:
        ax.plot(t, x, color=S.BLUE, lw=S.LW_CTX, drawstyle='steps-post')
    S.ref_line(ax, sh, 'static share', x=0.0, ha='left', va='top', dy=2)
    ax.annotate('measured pull', xy=(0.01, 0.98), xycoords='axes fraction', ha='left', va='top', fontsize=8,
                color=S.RED_MID)
    ax.annotate('planned', xy=(1.0, np.nanmean(pl[0][1])), xycoords=('axes fraction', 'data'), xytext=(0, 4),
                textcoords='offset points', ha='right', va='bottom', fontsize=8, color=S.BLUE)
    ax.set_xlim(0, max(t[-1] for t, _ in pl))
    ax.set_ylim(2, 5)
    ax.set_yticks([2, 3, 4, 5])
    ax.set_xlabel(r'$t$ from the path start (s)')
    ax.set_ylabel('Cable tension (N)')
    S.save(fig, 'fig_tension_eight')


if __name__ == '__main__':
    main()
