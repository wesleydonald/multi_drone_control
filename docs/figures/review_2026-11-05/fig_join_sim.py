#!/usr/bin/env python3
"""Attachment of the new agent in simulation (fig:res_join_sim; Wesley 9 Nov: draw the pending figure from a final run).

The one sentence: the collaborator's controller brings the new agent down onto its plate, the confirmation passes 1.5 s
after the contact, and the planner adds the fourth quadrotor, whose measured cable tension then rises while the others
share the change, with the ring within about 2 deg of level.
Hero (Wesley 9 Nov: (a) the measured tension of each of the four quadrotors, (b) the ring's tilt; the magnet-to-plate
panels dropped): (a) measured cable tension = drone mass x |the cable acceleration each tracker feeds forward|
(cable_source mocap, 0.2 s mean, as fig_release_rig), the new agent's only from the attachment (before it the
collaborator's controller flies it and its tracker measures no tension); (b) the ring's tilt with the number of
quadrotors the planner flies. One time axis from the new agent's takeoff, the collaborator's phases as light bands named
once on top.
Left out: the new agent's takeoff and transit (the window starts at its approach), the landing (the window ends 20 s
after the blend, before LAND; Q011's re-fly holds 30 s after the weld).
Data: RUN (default R4227, fri30_sq6b_m2a, layout (d) on Setup 3's ring, F097/F098; was R4106 on the 1.15 kg ring):
run.csv (payload_tilt_deg), events.csv (PHASE:*, CONTACT, WELD, LAND), the trackers' logs (acx/acy/acz, sim_time on the
run clock by the manifest's t_ready_sim), the planner log's n (as fig_s8_chain).
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import review_style as S  # noqa: E402

S.HERE = HERE
RUN = os.environ.get('MDC_JOIN_RUN', 'R4227')  # Q011: Setup 3's ring, LAND 30 s after the weld (was R4106, 1.15 kg)
MASS = 0.55
SMOOTH_S = 0.2
NEW_ID = 3                          # the new agent (QUAD4 is drone 3)


def run_dir(rid):
    (d,) = glob.glob(os.path.join(S.REPO, 'results', '2026-11-0*', f'{rid}_*'))
    return d


def main():
    d = run_dir(RUN)
    r = pd.read_csv(os.path.join(d, 'logs', 'run.csv'), usecols=['t', 'payload_tilt_deg']).drop_duplicates('t')
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    ev = ev[pd.to_numeric(ev.sim_time, errors='coerce').notna()].copy()
    ev['sim_time'] = ev.sim_time.astype(float)

    def at(name):
        return float(ev.loc[ev.event == name, 'sim_time'].iloc[0])

    t0 = at('PHASE:TAKEOFF')                     # the new agent's takeoff
    phases = [('Approach', at('PHASE:APPROACH_ABOVE_TARGET')), ('Descent', at('PHASE:DESCEND_TO_ATTACHMENT')),
              ('Confirmation', at('CONTACT')), ('Attached', at('WELD'))]
    t_land = at('LAND')
    t_contact, t_weld = at('CONTACT') - t0, at('WELD') - t0
    t_ready = float(json.load(open(os.path.join(d, 'manifest.json')))['t_ready_sim'])
    (pf,) = glob.glob(os.path.join(d, 'logs', 'dissipative_planner', '*', 'log.csv'))
    pl = pd.read_csv(pf, usecols=['sim_time', 'n'])
    t_n4 = float((pl.sim_time - t_ready)[pl.n >= 4].iloc[0]) - t0
    # the window ends 20 s after the blend (attach_traj_hold_s 10 s), before LAND (Wesley 9 Nov: the steady state, no landing)
    w0, w1 = phases[0][1] - t0, min(t_land - t0 - 0.5, t_weld + 30.0)
    if w1 < t_weld + 25.0:
        print(f'WARNING: LAND at +{t_land - t0:.1f} s, only {w1 - t_weld:.1f} s after the weld: no steady state in this run')

    S.setup()
    fig = S.figure('full', 3.0)
    a, c = fig.subplots(2, 1, sharex=True, gridspec_kw={'height_ratios': [1.35, 1.0]})
    edges = [p[1] - t0 for p in phases] + [w1]
    for ax in (a, c):
        ax.axvspan(t_contact, t_weld, color=S.FILL, lw=0, zorder=0)
        for te in edges[1:-1]:
            ax.axvline(te, color=S.LIGHT, lw=0.6, zorder=0)
    for k, (name, _) in enumerate(phases):
        narrow = name == 'Confirmation'                  # 1.5 s wide: its name starts at the band instead
        a.annotate(name, xy=(edges[k] if narrow else 0.5 * (edges[k] + edges[k + 1]), 1.0),
                   xycoords=('data', 'axes fraction'), xytext=(2 if narrow else 0, -3), textcoords='offset points',
                   ha='left' if narrow else 'center', va='top', fontsize=8, color='0.3')

    lines, labels = [], []
    for f in sorted(glob.glob(os.path.join(d, 'logs', 'tracker', 'drone*', 'log.csv'))):
        k = int(os.path.basename(os.path.dirname(f))[5])
        tr = pd.read_csv(f, usecols=['sim_time', 'acx', 'acy', 'acz'])
        tt = tr.sim_time.to_numpy(float) - t_ready - t0
        ten = MASS * np.sqrt(tr.acx ** 2 + tr.acy ** 2 + tr.acz ** 2).to_numpy(float)
        dt = np.nanmedian(np.diff(tt))
        ten = pd.Series(ten).rolling(max(1, int(round(SMOOTH_S / dt))), center=True, min_periods=1).mean().to_numpy()
        m = (tt >= max(w0, t_weld if k == NEW_ID else w0)) & (tt <= w1)
        line, = a.plot(tt[m], ten[m], color=S.ps.drone_colour(k), lw=S.LW_HERO if k == NEW_ID else S.LW_CTX + 0.2,
                       ls={1: (0, (4, 1.5)), 2: (0, (1, 1.2))}.get(k, '-'))     # a second cue for greyscale print
        lines.append(line)
        labels.append('New agent' if k == NEW_ID else f'Quadrotor {k + 1}')
        print(f'drone {k}: tension at the weld {np.interp(t_weld, tt, ten):.2f} N, at LAND {np.interp(w1, tt, ten):.2f} N')
    a.set_ylabel('Measured cable\ntension (N)')
    a.set_ylim(0, 5)                                     # room for the phase names inside the top
    a.set_yticks([0, 2, 4])

    m = (r.t - t0 >= w0) & (r.t - t0 <= w1)
    t = r.t[m].to_numpy() - t0
    tilt = r.payload_tilt_deg[m].to_numpy()
    after = t >= t_weld
    print(f'takeoff {t0:.2f}; contact +{t_contact:.2f}, weld +{t_weld:.2f}, planner n=4 +{t_n4:.2f}, LAND +{w1:.2f}; '
          f'tilt before {tilt[~after].max():.2f}, after peak {tilt[after].max():.2f} deg')
    c.plot(t, tilt, color=S.INK, lw=S.LW_HERO)
    c.set_ylabel(r'Ring tilt ($^\circ$)')
    top = max(2.5, float(np.ceil(tilt.max() + 0.6)))
    c.set_ylim(0, top)
    c.annotate('three quadrotors', xy=(w0 + 0.4, top - 0.3), fontsize=8, color=S.GREY, ha='left', va='center')
    c.annotate('four quadrotors', xy=(t_n4 + 0.4, top - 0.3), fontsize=8, color=S.GREY, ha='left', va='center')
    c.set_xlim(w0, w1)
    c.set_xticks([x for x in range(5, int(w1) + 1, 5) if x >= w0])
    c.set_xlabel("Time after the new agent's takeoff (s)")
    S.panel(a, 'a', 'layout (d) in hover')
    S.panel(c, 'b')
    fig.align_ylabels((a, c))
    order = sorted(range(len(lines)), key=lambda i: (labels[i] == 'New agent', labels[i]))
    S.ps.legend_below(fig, [lines[i] for i in order], [labels[i] for i in order], ncol=4)
    S.save(fig, 'fig_join_sim')


if __name__ == '__main__':
    main()
