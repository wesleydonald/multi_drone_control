#!/usr/bin/env python3
"""The rig: one of four carriers lets go unannounced (RIG-1008-r303_4, 8 Oct, 0.55 m cables), with the tension each
quad MEASURES (Wesley 27 Oct: a real-world release, no planned line).

Hero: measured cable tension per quad = drone mass (0.55 kg, the flight's) x |the cable acceleration the tracker fed
forward| (cable_source mocap: measured from motion capture, capped at 12 m/s^2 = 6.6 N; 0.2 s mean); survivors blue, the
released carrier red; the ring's tilt and height error below. Tracker logs moved onto the flight's clock by the offset
that matches their pose to run.csv (all four agree to 0.01 s).
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import review_style as S  # noqa: E402

RIG = os.path.join(S.REPO, 'results', 'rig', '2026-10-08')
FLIGHT = 'r303_4'
MASS = 0.55
SMOOTH_S = 0.2
WINDOW = (-3.0, 7.0)


def clock_offset(tr, run, k):
    rt, rx = run.t.to_numpy(), run[f'd{k}_x'].to_numpy()
    st, px = tr.sim_time.to_numpy(), tr.pose_x.to_numpy()
    best = (np.inf, 0.0)
    for off in np.arange(st.min() - rt.max(), st.max() - rt.min(), 0.01):
        x = np.interp(st - off, rt, rx, left=np.nan, right=np.nan)
        m = np.isfinite(x) & np.isfinite(px)
        if m.sum() > 100:
            e = np.mean((x[m] - px[m]) ** 2)
            if e < best[0]:
                best = (e, off)
    return best[1]


def main():
    run = pd.read_csv(os.path.join(RIG, FLIGHT + '_run', 'logs', 'run.csv'))
    ev = pd.read_csv(os.path.join(RIG, FLIGHT + '_run', 'logs', 'events.csv'))
    t_rel = float(ev.loc[ev.event == 'RELEASE', 'sim_time'].iloc[0])
    released = int(ev.loc[ev.event == 'RELEASE', 'arg'].iloc[0].split('/drone_')[1].rstrip(')'))
    t = run.t.to_numpy() - t_rel
    S.setup()
    fig = S.figure('full', 3.6)
    a, b, c = fig.subplots(3, 1, sharex=True, gridspec_kw={'height_ratios': [1.6, 1, 1]})
    for f in sorted(glob.glob(os.path.join(RIG, FLIGHT + '_logs', 'logs', 'tracker', 'drone*', 'log.csv'))):
        k = int(os.path.basename(os.path.dirname(f))[5])
        tr = pd.read_csv(f)
        off = clock_offset(tr.iloc[::5], run, k)
        tt = tr.sim_time.to_numpy() - off - t_rel
        ten = MASS * np.sqrt(tr.acx ** 2 + tr.acy ** 2 + tr.acz ** 2)
        dt = np.nanmedian(np.diff(tt))
        ten = pd.Series(ten).rolling(max(1, int(round(SMOOTH_S / dt))), center=True, min_periods=1).mean().to_numpy()
        m = (tt >= WINDOW[0]) & (tt <= WINDOW[1])
        a.plot(tt[m], ten[m], color=S.RED if k == released else S.BLUE, lw=S.LW_HERO if k == released else S.LW_CTX)
    a.annotate('released carrier', xy=(0.02, 0.06), xycoords='axes fraction', fontsize=8, color=S.RED)
    a.annotate('three survivors', xy=(0.02, 0.86), xycoords='axes fraction', fontsize=8, color=S.BLUE)
    a.set_ylabel('Measured\ntension (N)')
    a.set_ylim(0, 7)
    m = (t >= WINDOW[0]) & (t <= WINDOW[1])
    b.plot(t[m], run.payload_tilt_deg.to_numpy()[m], color=S.INK, lw=S.LW_HERO)
    b.set_ylabel(r'Ring tilt ($^\circ$)')
    b.set_ylim(0, 10)
    dz = 100.0 * (run.payload_z.to_numpy() - run.payload_ref_z.to_numpy())
    c.plot(t[m], dz[m], color=S.INK, lw=S.LW_HERO)
    c.axhline(0, color=S.LIGHT, lw=0.5, zorder=0)
    c.set_ylabel('Height\nerror (cm)')
    for ax in (a, b, c):
        ax.axvline(0, color=S.LIGHT, lw=0.6, zorder=0)
    a.annotate('release', xy=(0, 1.0), xycoords=('data', 'axes fraction'), xytext=(3, -2), textcoords='offset points',
               ha='left', va='top', fontsize=8, color=S.GREY)
    c.set_xlim(*WINDOW)
    c.set_xlabel('Time after the release (s)')
    S.save(fig, 'fig_release_rig')


if __name__ == '__main__':
    main()
