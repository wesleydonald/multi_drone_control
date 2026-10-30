#!/usr/bin/env python3
"""A carrier released from the T flies back to its own plate and is folded in, simulation (redesign of the 23 Oct
fig_rejoin; card docs/experiments/2026-10-23_rejoin_departed.md, E2; run R2594).

Message: the released carrier flies back to its own plate, its cable's end is lowered onto the plate and caught, it
is folded in, and the ring stays level (well under the 5 deg bar) throughout.
Hero (a): a 3-D view of the T from one fixed camera, the rejoiner drawn at five numbered moments (1 released,
2 staged, 3 over its plate, 4 caught, 5 folded in) with its cable as logged, its path between them coloured by its
role (carrying blue, departed grey, rejoining red); the other two carriers and the ring at the catch.
Support (b)-(d), sharing the numbered moments: the distance from the cable's end to its plate point (the rod's pose
in the simulation's state log), the number of carriers (the planner's n), the ring's tilt against the 5 deg bar.
Left out: the lift and the landing, the moments before the release.
Data: R2594 (rj3_pairrelease_rejoin_L100_tip): run.csv (ring and UAVs), events.csv (the moments), the planner log
(n, moved onto the run clock by t_ready_sim), gz_record/state.tlog (the rods), via tools/snapshot_render.py.
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from review30_style import S, TEXT  # noqa: E402

sys.path.insert(0, S.TOOLS)
import snapshot_render as SR  # noqa: E402

RUN, D = 'R2594', 1
TILT_BAR_DEG = 5.0
SUBJ = S.RED                      # the rejoiner, everywhere
PALE = '#e3a3a3'                  # its cables, so its path reads first
CTX = '0.45'                      # the two carriers that stay
AZIM, ELEV = 0.0, 10.0             # from +x, just above: the rejoiner flies in the y-z plane
PRE, POST = 8.0, 5.0              # s shown before the release and after the fold-in
NAMES = ('released', 'staged', 'over its plate', 'caught', 'folded in')


def moments(run):
    ev = {e: t for t, e, a in run.events}
    t, x, y = run.t, run.df[f'd{D}_x'].to_numpy(), run.df[f'd{D}_y'].to_numpy()
    plate = run.plate_world(ev['REJOIN_WELD'], D)
    over = (t > ev['REJOIN_START']) & (t < ev['REJOIN_WELD']) & (np.hypot(x - plate[0], y - plate[1]) < 0.02)
    return ev, [ev['RELEASE'], ev['REJOIN_START'], float(t[over][0]), ev['REJOIN_WELD'], ev['REJOIN_BLEND_OVER']]


def badge(ax, xy, k, off=(0, 0), coords='data', text=None, ha='center'):
    ax.annotate(str(k), xy=xy, xycoords=coords, xytext=off, textcoords='offset points', ha='center', va='center',
                fontsize=7.5, color='white', zorder=20, bbox=dict(boxstyle='circle,pad=0.18', fc=SUBJ, ec='none'))
    if text:
        dx = off[0] + (7 if ha == 'left' else -7)
        ax.annotate(text, xy=xy, xycoords=coords, xytext=(dx, off[1]), textcoords='offset points', ha=ha,
                    va='center', fontsize=8, color=TEXT, zorder=20)


def disc(ax, cam, p, colour, z):
    d = cam(SR._circle(p, np.eye(3), 0.085, 40))
    ax.fill(d[:, 0], d[:, 1], facecolor=colour, edgecolor='white', lw=0.5, zorder=z)


def main():
    (d,) = glob.glob(os.path.join(S.REPO, 'results', '*', f'{RUN}_*'))
    run = SR.Run(d)
    ev, tm = moments(run)
    t0 = tm[0]

    S.setup()
    fig = S.figure('full', 3.2)
    gs = fig.add_gridspec(3, 2, width_ratios=[1.0, 1.1], height_ratios=[1.0, 0.36, 0.8])
    ax = fig.add_subplot(gs[:, 0])
    ax_d = fig.add_subplot(gs[0, 1])
    ax_n = fig.add_subplot(gs[1, 1], sharex=ax_d)
    ax_t = fig.add_subplot(gs[2, 1], sharex=ax_d)

    # (a) one camera: the ring and the two carriers that stay at the catch, the rejoiner at the five moments
    cam = SR.Camera(AZIM, ELEV)
    t_c = tm[3]
    c, rm = run.ring(t_c)
    pts = []
    for i in range(run.n):
        if i == D:
            continue
        top, end = run.cable(t_c, i)
        seg = cam(np.vstack([top, end]))
        ax.plot(seg[:, 0], seg[:, 1], color=CTX, lw=0.6, zorder=3)
        disc(ax, cam, run.uav(t_c, i), CTX, 8 + float(cam.depth(run.uav(t_c, i))[0]))
        pts += [top, end]
    ring = SR._circle(c, rm, run.ring_r)
    uv, far = cam(ring), (cam.depth(ring) - cam.depth(c)[0]) < 0
    for mask, col, z in ((far, '0.6', 2), (~far, 'black', 6)):
        k = np.where(mask, 1.0, np.nan)
        ax.plot(uv[:, 0] * k, uv[:, 1] * k, color=col, lw=1.6, zorder=z)
    pl = cam(c + (rm @ run.plates.T).T)
    ax.plot(pl[:, 0], pl[:, 1], 'o', ms=2.4, color='black', zorder=7)
    ax.plot(pl[D, 0], pl[D, 1], 'o', ms=3.4, color=SUBJ, zorder=7)       # its own plate
    pts += list(ring)

    tt = run.t
    body = run.df[[f'd{D}_x', f'd{D}_y', f'd{D}_z']].to_numpy(float)
    out = (tt >= tm[0]) & (tt <= tm[1])
    back = (tt >= tm[1]) & (tt <= tm[4])
    p = cam(body[out])
    ax.plot(p[:, 0], p[:, 1], color=SUBJ, lw=0.9, ls=(0, (2.5, 1.8)), zorder=5)
    p = cam(body[back])
    ax.plot(p[:, 0], p[:, 1], color=SUBJ, lw=S.LW_HERO, zorder=5)
    # one arrowhead per leg, at its middle: out, climb and transit, descent, fold-in
    legs = ((tm[0], tm[1], 0.5), (tm[1], tm[2], 0.55), (tm[2], tm[3], 0.75), (tm[3], tm[4], 0.5))
    for a, b, f in legs:
        w = np.flatnonzero((tt >= a) & (tt <= b))
        q = cam(body[w])
        L = np.r_[0, np.cumsum(np.hypot(*np.diff(q, axis=0).T))]
        i = int(np.searchsorted(L, f * L[-1]))
        j = int(np.searchsorted(L, f * L[-1] + 0.03))
        ax.annotate('', xy=q[min(j, len(q) - 1)], xytext=q[max(i - 1, 0)],
                    arrowprops=dict(arrowstyle='-|>', color=SUBJ, lw=0, mutation_scale=8, shrinkA=0, shrinkB=0),
                    zorder=6)
    pts += list(body[out | back])
    for k, t in enumerate(tm, start=1):
        top, end = run.cable(t, D)
        if k != 5:                                     # 5 is back where 1 was: one cable for both
            seg = cam(np.vstack([top, end]))
            ax.plot(seg[:, 0], seg[:, 1], color=PALE, lw=0.5, zorder=4)
            ax.plot(seg[1, 0], seg[1, 1], 'o', ms=2.6, mfc='white', mec=SUBJ, mew=0.6, zorder=6)
        disc(ax, cam, run.uav(t, D), SUBJ, 12)
        pts += [top, end]
    # badge and name per moment, offsets in points from the UAV; a white halo where a cable passes behind a name
    halo = dict(boxstyle='square,pad=0.08', fc='white', ec='none')
    place = {1: ((-9, -11), 'right', NAMES[0]), 5: ((9, -11), 'left', NAMES[4]), 2: ((-10, 0), 'right', NAMES[1]),
             3: ((0, 10), 'left', NAMES[2]), 4: ((10, 11), 'left', NAMES[3])}
    for k, (off, ha, text) in place.items():
        xy = cam(run.uav(tm[k - 1], D))[0]
        badge(ax, xy, k, off)
        tx = off[0] + {'right': -7, 'left': 7, 'center': 0}[ha]
        ty = off[1] + (9 if ha == 'center' else 0)
        ax.annotate(text, xy=xy, xytext=(tx, ty), textcoords='offset points', ha=ha, va='center', fontsize=8,
                    color=S.INK, zorder=19, bbox=halo)
    uv = cam(np.vstack(pts))
    lo, hi = uv.min(0), uv.max(0)
    pad = 0.07 * (hi - lo)
    ax.set_xlim(lo[0] - 4.6 * pad[0], hi[0] + 2.5 * pad[0])
    ax.set_ylim(lo[1] - 2.2 * pad[1], hi[1] + 2.2 * pad[1])
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    xl, yl = ax.get_xlim(), ax.get_ylim()
    a0 = np.array([xl[0] + 0.3 * (xl[1] - xl[0]), yl[0] + 0.05 * (yl[1] - yl[0])])
    ax.plot([a0[0], a0[0] + 0.5], [a0[1]] * 2, color='0.3', lw=1.0, solid_capstyle='butt')
    ax.annotate('0.5 m', xy=(a0[0] + 0.25, a0[1]), xytext=(0, 2), textcoords='offset points', ha='center',
                va='bottom', fontsize=8, color='0.3')

    # inset: the T from above, turned to match (a) (its right is +y, its up is -x: the camera stands at +x)
    ins = ax.inset_axes([0.04, 0.2, 0.27, 0.3])

    def top(p):
        p = np.atleast_2d(p)
        return p[:, 1], -p[:, 0]
    ins.plot(*top(SR._circle(c, rm, run.ring_r)), color='black', lw=1.0)
    plw = c + (rm @ run.plates.T).T
    ins.plot(*top(plw), 'o', ms=1.8, color='black')
    ins.plot(*top(plw[D]), 'o', ms=2.6, color=SUBJ)
    for i in range(run.n):
        u = run.uav(t_c, i) if i != D else run.uav(tm[0], D)
        ins.plot(*top(np.vstack([u, plw[i]])), color=CTX if i != D else PALE, lw=0.5)
        ins.plot(*top(u), 'o', ms=3.2, color=CTX if i != D else SUBJ, mec='white', mew=0.4)
    ins.plot(*top(body[out | back]), color=SUBJ, lw=0.8)
    ins.set_aspect('equal')
    ins.set_xlim(-1.2, 0.4)
    ins.set_ylim(-1.0, 1.0)
    ins.set_xticks([])
    ins.set_yticks([])
    for sp in ins.spines.values():
        sp.set_linewidth(0.5)
        sp.set_color('0.5')
    ins.annotate('from above', xy=(0.5, 1.0), xycoords='axes fraction', xytext=(0, 2), textcoords='offset points',
                 ha='center', va='bottom', fontsize=7, color=S.INK)

    # (b) the cable's end to its plate point (the rod as the state log has it)
    lo_t, hi_t = t0 - PRE, tm[4] + POST
    ts, ends = run.cable_end_series(D)
    dist = np.array([np.linalg.norm(e - run.plate_world(t, D)) for t, e in zip(ts, ends)])
    w = (ts >= lo_t) & (ts <= hi_t)
    ax_d.plot(ts[w] - t0, 100 * dist[w], color=SUBJ, lw=S.LW_CTX, rasterized=True)
    k4 = int(np.searchsorted(ts, tm[3]))
    ax_d.annotate(f'{100 * dist[k4]:.1f} cm', xy=(tm[3] - t0, 100 * dist[k4]), xytext=(4, 12),
                  textcoords='offset points', ha='left', va='bottom', fontsize=8, color=SUBJ,
                  arrowprops=dict(arrowstyle='-', color=SUBJ, lw=0.5, shrinkA=0, shrinkB=1))
    ax_d.set_ylim(0, 125)
    ax_d.set_yticks([0, 50, 100])
    ax_d.set_ylabel('End to plate\n(cm)')

    # (c) the number of carriers (the planner's OCP size)
    (pf,) = glob.glob(os.path.join(d, 'logs', 'dissipative_planner', '*', 'log.csv'))
    pl_log = pd.read_csv(pf, usecols=['sim_time', 'n'])
    tp = pl_log.sim_time.to_numpy() - run.t_ready
    w = (tp >= lo_t) & (tp <= hi_t)
    ax_n.step(tp[w] - t0, pl_log.n.to_numpy()[w], where='post', color=S.INK, lw=S.LW_HERO)
    ax_n.set_ylim(1.5, 3.5)
    ax_n.set_yticks([2, 3])
    ax_n.set_ylabel('Carriers')

    # (d) the ring's tilt
    tilt = run.df.payload_tilt_deg.to_numpy(float)
    w = (tt >= lo_t) & (tt <= hi_t)
    ax_t.plot(tt[w] - t0, tilt[w], color=S.INK, lw=S.LW_CTX, rasterized=True)
    S.ref_line(ax_t, TILT_BAR_DEG, r'$5^\circ$ bar', x=0.0, ha='left', va='bottom', dy=1.0)
    ax_t.set_ylim(0, 6.5)
    ax_t.set_yticks([0, 2, 4, 6])
    ax_t.set_ylabel(r'Ring tilt ($^\circ$)')
    ax_t.set_xlabel(r'$t$ after the release (s)')
    ax_t.set_xlim(-PRE, hi_t - t0)

    for k, t in enumerate(tm, start=1):
        for a in (ax_d, ax_n, ax_t):
            a.axvline(t - t0, color=S.LIGHT, lw=0.5, zorder=0)
        badge(ax_d, (t - t0, 1.0), k, (0, 8), coords=('data', 'axes fraction'))
    for a in (ax_d, ax_n):
        a.tick_params(labelbottom=False)
    fig.align_ylabels([ax_d, ax_n, ax_t])
    S.panel(ax, 'a')
    S.panel(ax_d, 'b')
    S.panel(ax_n, 'c')
    S.panel(ax_t, 'd')

    wfold = (tt >= ev['REJOIN_WELD']) & (tt <= ev['REJOIN_WELD'] + 15.0)
    wrel = (tt >= tm[0]) & (tt <= tm[1])
    print('moments (s after the release):', [round(t - t0, 2) for t in tm])
    print(f"cable end at the catch {100 * dist[np.searchsorted(ts, tm[3])]:.2f} cm; within 2 cm from "
          f"{ts[(ts > tm[1]) & (dist < 0.02)][0] - t0:.1f} s; tilt peak after the release {np.nanmax(tilt[wrel]):.2f} "
          f"deg, in the fold-in (weld..+15 s) {np.nanmax(tilt[wfold]):.2f} deg; weld "
          f"{ev['REJOIN_WELD'] - ev['REJOIN_START']:.2f} s after the rejoin command")
    S.save(fig, 'fig_rejoin')


if __name__ == '__main__':
    main()
