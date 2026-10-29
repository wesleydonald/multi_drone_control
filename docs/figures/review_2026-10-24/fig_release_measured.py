#!/usr/bin/env python3
"""One of four carriers lets go unannounced on 1 m cables (twin), with the tension each quad MEASURES (Wesley 27 Oct).

Message: the released carrier's pull drops to nothing and the three survivors take the ring's weight between them within
about a second; the ring tilts at most ~6 deg and is back near level in ~3 s, and dips ~3.5 cm.
Hero: measured cable tension per quad (0.55 kg x |acm|, the trackers' raw measured cable acceleration; 0.3 s mean),
survivors blue, the released carrier red, the planned share after the release grey dashed. Data: R2486 (r1m_ta_a1_r1).
"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402
import metrics as M  # noqa: E402

RUN = 'R2486'
DRONE_MASS = 0.55
SMOOTH_S = 0.3
WINDOW = (-5.0, 12.0)


def main():
    run = glob.glob(os.path.join(S.REPO, 'results', '2026-10-2*', RUN + '_*'))[0]
    man = json.load(open(os.path.join(run, 'manifest.json')))
    t_ready = float(man['t_ready_sim'])
    ev = pd.read_csv(os.path.join(run, 'logs', 'events.csv'))
    t_rel = float(ev.loc[ev.event == 'RELEASE', 'sim_time'].iloc[0])
    d = M.load_run(run)['data']
    t = np.asarray(d['t'], float) - t_rel
    tilt = np.asarray(d['payload_tilt_deg'], float)
    dz = 100.0 * (np.asarray(d['payload_z'], float) - np.asarray(d['payload_ref_z'], float))

    S.setup()
    fig = S.figure('full', 3.6)
    a, b, c = fig.subplots(3, 1, sharex=True, gridspec_kw={'height_ratios': [1.6, 1, 1]})
    for f in sorted(glob.glob(os.path.join(run, 'logs', 'tracker', 'drone*', 'log.csv'))):
        tr = pd.read_csv(f)
        tt = tr.sim_time.to_numpy() - t_ready - t_rel
        ten = DRONE_MASS * np.sqrt(tr.acm_x ** 2 + tr.acm_y ** 2 + tr.acm_z ** 2)
        dt = np.nanmedian(np.diff(tt))
        ten = pd.Series(ten).rolling(max(1, int(round(SMOOTH_S / dt))), center=True, min_periods=1).mean().to_numpy()
        released = np.nanmedian(ten[(tt > 2) & (tt < 8)]) < 1.0 if np.any((tt > 2) & (tt < 8)) else True
        m = (tt >= WINDOW[0]) & (tt <= WINDOW[1])
        if released:
            # its own landing (ground contact) reads as pull at the end of its log: stop before it climbs back
            back = np.where((tt > 2) & (ten > 0.3))[0]
            if len(back):
                m &= tt < tt[back[0]]
        a.plot(tt[m], ten[m], color=S.RED if released else S.BLUE, lw=S.LW_HERO if released else S.LW_CTX)
    a.axhline(3.28, color=S.GREY, lw=S.LW_REF, ls=(0, (4, 3)), zorder=0)
    a.annotate('planned share after the release, 3.28 N', xy=(1.0, 3.28), xycoords=('axes fraction', 'data'),
               xytext=(0, 3), textcoords='offset points', ha='right', va='bottom', fontsize=8, color=S.GREY)
    a.annotate('released carrier', xy=(0.02, 0.06), xycoords='axes fraction', fontsize=8, color=S.RED)
    a.annotate('three survivors', xy=(0.02, 0.86), xycoords='axes fraction', fontsize=8, color=S.BLUE)
    a.set_ylabel('Measured\ntension (N)')
    a.set_ylim(0, 5.5)
    m = (t >= WINDOW[0]) & (t <= WINDOW[1])
    b.plot(t[m], tilt[m], color=S.INK, lw=S.LW_HERO)
    b.set_ylabel(r'Ring tilt ($^\circ$)')
    b.set_ylim(0, 8)
    c.plot(t[m], dz[m], color=S.INK, lw=S.LW_HERO)
    c.axhline(0, color=S.LIGHT, lw=0.5, zorder=0)
    c.set_ylabel('Height\nerror (cm)')
    c.set_ylim(-5, 2)
    c.set_yticks([-4, -2, 0, 2])
    for ax in (a, b, c):
        ax.axvline(0, color=S.LIGHT, lw=0.6, zorder=0)
    a.annotate('release', xy=(0, 1.0), xycoords=('data', 'axes fraction'), xytext=(3, -2), textcoords='offset points',
               ha='left', va='top', fontsize=8, color=S.GREY)
    c.set_xlim(*WINDOW)
    c.set_xlabel('Time after the release (s)')
    S.save(fig, 'fig_release_measured')


if __name__ == '__main__':
    main()
