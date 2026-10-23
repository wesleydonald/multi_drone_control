#!/usr/bin/env python3
"""Reaction to an unannounced release on the rig: four carriers in hover, the operator opens one magnet (8 Oct).

Message: the planner sees the released cable stretch past its 6 cm threshold 0.42 s after the magnet opens, resizes to
three carriers 0.09 s later (their planned tensions step from 1.6-3.8 N to about 3.2-3.3 N each), and the ring's tilt peaks
at 6.5 deg and is back in its band 1.8 s after the release.
Hero: three panels on one clock from the release (the operator's magnet OFF in the bag): (a) the released carrier's
rod stretch (red), pivot to plate, against the detection threshold, with the detection time bracketed; (b) the planned
tension of each cable, the released carrier red, the survivors blue; (c) the ring's tilt (black: the ring beside the
drones' red and blue), its band (the 5 s before the release, peak + 0.5 deg) and the time to be back in it bracketed. Events: RELEASE, DETECTED (the planner's UNANNOUNCED DETACH line) and RESIZE
(its DETACH, the OCP resized to n = 3), from the T2 console via events.csv.
The stretch is computed at 50 Hz as the planner computes it (|ring + R rho_i - (drone + pivot)|, rho on the 0.225 m
rim at the leaver's slot azimuth; the drone taken level for its 4 cm pivot offset): 2-4 mm from the planner's own
10 Hz len column before the release. Window: 1 s before to 6 s after the release (LAND came 7.6 s after it).
"""
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import best_style as S  # noqa: E402
import best_data as B  # noqa: E402

T0, T1 = -1.0, 6.0


def stretch(f, d, drone, az_deg):
    q = d[['payload_qx', 'payload_qy', 'payload_qz', 'payload_qw']].to_numpy()
    ok = np.all(np.isfinite(q), axis=1)
    R = Rot.from_quat(np.where(ok[:, None], q, [0, 0, 0, 1]))
    r = float(f.params['attach_radius'])
    rho = np.array([r * np.cos(np.radians(az_deg)), r * np.sin(np.radians(az_deg)), float(f.params['attach_z'])])
    att = d[['payload_x', 'payload_y', 'payload_z']].to_numpy() + R.apply(rho)
    piv = d[[f'd{drone}_x', f'd{drone}_y', f'd{drone}_z']].to_numpy() + np.array([0, 0, -0.04])
    out = np.linalg.norm(att - piv, axis=1) - float(f.params['cable_len'])
    return np.where(ok, out, np.nan)


def bracket(ax, x0, x1, y, text, left=False):
    ax.annotate('', xy=(x0, y), xytext=(x1, y), arrowprops=dict(arrowstyle='<->', lw=0.6, color=S.GREY,
                                                                 shrinkA=0, shrinkB=0))
    ax.annotate(text, xy=(x0, y) if left else (x1, y), xytext=(-3 if left else 3, 0), textcoords='offset points',
                ha='right' if left else 'left', va='center', fontsize=8, color=S.GREY)


def main():
    f = B.Flight(B.RELEASE)
    d = f.run()
    t = d.t.to_numpy()
    tr = f.t_event('RELEASE')
    t_det = f.t_event('DETECTED', tr) - tr
    t_rs = f.t_event('DETACH', tr) - tr
    x = t - tr
    win = (x >= T0) & (x <= T1)
    leaver = int(next(a for _, e, a in f.events() if e == 'RELEASE').split('/drone_')[1].rstrip(')'))
    p = f.planner()
    s2d0 = [int(v) for v in str(p.slot2drone[p.t < tr].iloc[-1]).split()]
    az = [float(a) for a in str(f.params['attach_azimuths_deg']).split(',')][s2d0.index(leaver)]
    st = stretch(f, d, leaver, az)

    # planned tension per drone: t<slot> mapped through each row's slot2drone
    px = p.t.to_numpy() - tr
    ten = {k: np.full(len(p), np.nan) for k in range(f.n)}
    for j, (s2d, row) in enumerate(zip(p.slot2drone.astype(str), p.itertuples())):
        for slot, dr in enumerate(int(v) for v in s2d.split()):
            ten[dr][j] = getattr(row, f't{slot}')
    tilt = d.payload_tilt_deg.to_numpy()
    pre = (x >= -5) & (x < 0)
    band = float(np.nanmax(tilt[pre])) + 0.5
    post = (x >= 0) & (x <= T1)
    k = int(np.nanargmax(np.where(post, tilt, np.nan)))
    back = float(x[np.flatnonzero(post & (tilt > band))[-1]])
    after = (px > t_rs + 0.3) & (px < T1)
    print(f'detected +{t_det:.2f} s, resize +{t_rs:.2f} s, tilt peak {tilt[k]:.2f} deg at +{x[k]:.2f} s, band '
          f'{band:.2f} deg, back in band +{back:.2f} s; leaver slot az {az:.0f} deg; survivors after: '
          + ' '.join(f'{np.nanmean(ten[dr][after]):.2f}' for dr in ten if dr != leaver))

    S.setup()
    fig = S.figure('full', 3.5)
    axs = fig.subplots(3, 1, sharex=True)
    for ax in axs:
        ax.axvline(0, color=S.LIGHT, lw=0.5, zorder=0)
        ax.axvline(t_det, color=S.LIGHT, lw=0.5, zorder=0)
        ax.axvline(t_rs, color=S.LIGHT, lw=0.5, zorder=0)
    axs[0].annotate('RELEASE', xy=(0, 1), xycoords=('data', 'axes fraction'), xytext=(-2, 2),
                    textcoords='offset points', ha='right', va='bottom', fontsize=8, color=S.GREY)
    axs[0].annotate('DETECTED, RESIZE', xy=(t_rs, 1), xycoords=('data', 'axes fraction'), xytext=(2, 2),
                    textcoords='offset points', ha='left', va='bottom', fontsize=8, color=S.GREY)

    # (a) the released rod's stretch
    cut = win & (x <= t_rs + 0.25)                 # after the resize the drone is free: no longer a stretch
    axs[0].plot(x[cut], 100 * st[cut], color=S.RED, lw=S.LW_HERO)
    S.ref_line(axs[0], 100 * float(f.params['detach_detect_m']), 'threshold', x=1.0, ha='right', va='bottom')
    bracket(axs[0], 0, t_det, 13, f'{t_det:.2f}\\,s', left=True)
    axs[0].set_ylim(-2, 20)
    axs[0].set_yticks([0, 10, 20])
    axs[0].set_ylabel('Stretch (cm)')

    # (b) planned tensions
    for dr, v in ten.items():
        m = (px >= T0) & (px <= T1)
        lv = dr == leaver
        axs[1].plot(px[m], v[m], drawstyle='steps-post', color=S.RED if lv else S.BLUE, lw=S.LW_CTX)
    last = np.flatnonzero(np.isfinite(ten[leaver]) & (px >= T0))[-1]
    axs[1].annotate('released', xy=(px[last], ten[leaver][last]), xytext=(-4, -3), textcoords='offset points',
                    ha='right', va='top', fontsize=8, color=S.RED)
    axs[1].annotate('survivors', xy=(T1, np.nanmean([ten[dr][after][-1] for dr in ten if dr != leaver])),
                    xytext=(0, 4), textcoords='offset points', ha='right', va='bottom', fontsize=8, color=S.BLUE)
    axs[1].set_ylim(0, 5)
    axs[1].set_yticks([0, 2, 4])
    axs[1].set_ylabel('Cable tension (N)')

    # (c) ring tilt
    axs[2].plot(x[win], tilt[win], color=S.FLEET_COLOUR[4], lw=S.LW_HERO)
    axs[2].axhline(band, color=S.GREY, lw=0.6, ls=':')
    axs[2].annotate('band', xy=(1, band), xycoords=('axes fraction', 'data'), xytext=(0, 2), textcoords='offset points',
                    ha='right', va='bottom', fontsize=8, color=S.GREY)
    axs[2].annotate(f'{tilt[k]:.1f}' + r'$^\circ$', xy=(x[k], tilt[k]), xytext=(-4, 0), textcoords='offset points',
                    ha='right', va='center', fontsize=8, color=S.INK)
    bracket(axs[2], 0, back, 9.0, f'{back:.1f}\\,s')
    axs[2].set_ylim(0, 10)
    axs[2].set_yticks([0, 5, 10])
    axs[2].set_ylabel(r'Ring tilt ($^\circ$)')
    axs[2].set_xlim(T0, T1)
    axs[2].set_xlabel(r'$t$ after the release (s)')
    fig.align_ylabels(axs)
    for ax, letter in zip(axs, 'abc'):
        S.ps.panel_label(ax, letter)
    S.save(fig, 'fig_release')


if __name__ == '__main__':
    main()
