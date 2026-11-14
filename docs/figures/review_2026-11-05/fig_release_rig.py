#!/usr/bin/env python3
"""5 Nov revision of review_2026-10-24/fig_release_rig.py (Wesley: figures "extremely high quality"): panel labels, the band
and the time back in it drawn (fig_reaction.py's definition, the one the table uses), the detection marked from
events.csv, plain legend words (remaining / released quadrotor).

The one sentence: the planner detects the release from motion capture and the ring is back in its band 1.8 s later.

The real world: one of four carriers lets go unannounced (RIG-1008-r303_4, 8 Oct, 0.55 m cables), with the tension each
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
    fig = S.figure('full', 3.95)        # panels, their (a)-(c) labels and the legend below
    a, b, c = fig.subplots(3, 1, sharex=True, gridspec_kw={'height_ratios': [1.6, 1, 1]})
    key = {}
    for f in sorted(glob.glob(os.path.join(RIG, FLIGHT + '_logs', 'logs', 'tracker', 'drone*', 'log.csv'))):
        k = int(os.path.basename(os.path.dirname(f))[5])
        tr = pd.read_csv(f)
        off = clock_offset(tr.iloc[::5], run, k)
        tt = tr.sim_time.to_numpy() - off - t_rel
        ten = MASS * np.sqrt(tr.acx ** 2 + tr.acy ** 2 + tr.acz ** 2)
        dt = np.nanmedian(np.diff(tt))
        ten = pd.Series(ten).rolling(max(1, int(round(SMOOTH_S / dt))), center=True, min_periods=1).mean().to_numpy()
        m = (tt >= WINDOW[0]) & (tt <= WINDOW[1])
        line, = a.plot(tt[m], ten[m], color=S.RED if k == released else S.INK,
                       lw=S.LW_HERO if k == released else S.LW_CTX)
        key['Detached quadrotor' if k == released else 'Remaining quadrotors'] = line
    a.set_ylabel('Measured cable\ntension (N)')
    a.set_ylim(0, 6.2)
    a.set_yticks([0, 2, 4, 6])
    m = (t >= WINDOW[0]) & (t <= WINDOW[1])
    tilt = run.payload_tilt_deg.to_numpy()
    t_det = float(ev.loc[ev.event == 'DETECTED', 'sim_time'].iloc[0]) - t_rel
    t_land = float(ev.loc[ev.event == 'LAND', 'sim_time'].iloc[0]) - t_rel
    band = float(np.nanmax(tilt[(t >= -5) & (t < 0)])) + 0.5          # fig_reaction.py: highest tilt before + 0.5 deg
    post = (t >= 0) & (t <= min(t_land, 15.0))
    back = float(t[np.flatnonzero(post & (tilt > band))[-1]])
    b.plot(t[m], tilt[m], color=S.INK, lw=S.LW_HERO)
    # Wesley 14 Nov: the recovery band and its bracket are left to the text (band and time still printed below)
    b.set_ylabel(r'Payload tilt ($^\circ$)')
    b.set_ylim(0, 8.2)
    b.set_yticks([0, 2, 4, 6, 8])
    dz = 1000.0 * (run.payload_z.to_numpy() - run.payload_ref_z.to_numpy())
    c.plot(t[m], dz[m], color=S.INK, lw=S.LW_HERO)
    c.axhline(0, color=S.LIGHT, lw=0.5, zorder=0)
    c.set_ylabel('Payload height\nerror (mm)')
    c.set_yticks([-30, 0, 30])
    for ax in (a, b, c):
        ax.axvline(0, color=S.LIGHT, lw=0.6, zorder=0)
        ax.axvline(t_det, color=S.LIGHT, lw=0.6, zorder=0)
    a.annotate('Magnet off', xy=(0, 1.0), xycoords=('data', 'axes fraction'), xytext=(-3, -2), textcoords='offset points',
               ha='right', va='top', fontsize=8, color=S.INK)
    a.annotate(f'Detected, {t_det:.2f} s', xy=(t_det, 1.0), xycoords=('data', 'axes fraction'), xytext=(3, -2),
               textcoords='offset points', ha='left', va='top', fontsize=8, color=S.INK)
    for ax, letter, name in zip((a, b, c), 'abc', ('Measured tension', 'Payload tilt', 'Payload height error')):
        S.panel(ax, letter, name)
    fig.align_ylabels((a, b, c))
    c.set_xlim(*WINDOW)
    c.set_xlabel('Time after the magnet is switched off (s)')
    S.ps.legend_below(fig, [key['Remaining quadrotors'], key['Detached quadrotor']],
                      ['Remaining quadrotors', 'Detached quadrotor'], ncol=2)
    print(f'band {band:.2f} deg, back in band {back:.2f} s, detected {t_det:.2f} s, LAND {t_land:.2f} s')
    S.save(fig, 'fig_release_rig')


if __name__ == '__main__':
    main()
