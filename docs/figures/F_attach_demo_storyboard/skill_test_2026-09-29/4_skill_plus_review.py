#!/usr/bin/env python3
"""Attach during a circle in Gazebo (R0209): three snapshots, plan view, ring tilt.

    python3 storyboard.py [RUN_DIR] [--out STEM]

Brief (thesis-figure skill, step 1)
-----------------------------------
1. Sentence. With three drones on one side the ring flies the circle tilted by about
   35 deg; after the newcomer welds to the stationary ring at its empty 6 o'clock rim,
   the four-drone fleet levels it to under 10 deg and completes the circle.
2. Hero. (c) ring tilt against time: the widest plot and the only data trace with
   weight. It carries 35 deg -> under 10 deg, how long after WELD, and that it stays
   there to the end of the circle. Support: (a) three snapshots at ATTACH, WELD and
   RESUME, drawn from the logged poses and seen from beside the tilt axis, so the tilt
   reads as a see-saw against a level reference (the empty 6 o'clock side hangs, the
   newcomer fills it, the ring levels), and (b) the plan view (where: the circle, the
   hold, the newcomer's approach ending on the 6 o'clock rim, the loop flown by four).
3. Left out. The payload target in the plan (the ring path already draws the circle,
   and the target's re-anchoring step at the weld needed explaining without serving
   the sentence), carrier paths, the newcomer before its approach, heights, tensions,
   tracking errors, everything after LAND, a legend. Carrier colours are the drone
   palette and are keyed in (b), where the ring at the weld carries each drone's rim
   anchor at its clock position.
4. Window. The target's sweep (metrics.sweep_window) start to LAND (events.csv).
5. Caption draft. "Attach during a circle in Gazebo (R0209, 0.6 kg ring of radius
   0.25 m on 0.5 m rods, 0.5 m circle at 0.2 m/s; dissipative network). (a) The fleet
   at ATTACH, WELD and RESUME, drawn from the logged ring pose and drone positions,
   seen from east-north-east so that 6 o'clock is on the left (thick: near half of the
   ring; dotted: the ring if it were level; numerals: the empty 6 o'clock rim and the
   12 o'clock rim opposite it; bar: 0.25 m). (b) Plan view of the ring centre (open
   circle: start; black dots: ATTACH and RESUME; arrow: direction of travel) and of the
   newcomer's approach from ATTACH to WELD (red; it starts 1.2 m south of the start,
   below the crop), with the ring outline at the weld and its four rim anchors in the
   drones' colours (12 o'clock = +y). (c) Ring tilt, to LAND; grey band: the 20 s for
   which the target is held; dashed: 10 deg; arrow: WELD to the last sample above
   10 deg. Three drones at 3, 12 and 9 o'clock fly with the
   ring tilted by about 35 deg; ATTACH freezes the payload target; the newcomer welds
   to the stationary ring at its 6 o'clock rim; the four drones level the ring, which
   stays under 10 deg from 9.3 s after the weld; the trajectory resumes and the
   four-drone fleet completes the circle. In the hover after the circle a 3 Hz tilt
   oscillation grows to 9 deg before LAND."

Every number drawn comes from the run directory: run.csv (poses, tilt, target, attach
flag), events.csv (ATTACH = MAGNET, WELD, LAND), launch.log (the reconfiguration hold,
hence RESUME, and the weld offset on the rim), manifest.json (rim azimuths of the
carriers) and params/dissipative_controller.yaml (rim radius). The only typed number is
the 10 deg level of the claim itself.
"""
import argparse
import os
import re
import sys

import numpy as np

REPO = '/home/wesley/multi_drone_control'
sys.dont_write_bytecode = True                  # read the repo, write nothing into it
sys.path.insert(0, os.path.join(REPO, 'tools'))
import plot_style as ps                                   # noqa: E402
import metrics                                            # noqa: E402
import matplotlib.pyplot as plt                           # noqa: E402

RUN = os.path.join(REPO, 'results_archive', 'R0209_sim_gz_attach_circle_n3_fast')
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'storyboard')

LEVEL_DEG = 10.0            # the caption's claim: "under 10 deg"

# Snapshot camera: azimuth of the viewer about z and elevation above the horizon (deg).
# Nearly along the ring's tilt axis (x), so the 35 deg tilt is a rotation on the page;
# 20 deg off it so the 3 and 9 o'clock drones do not stand on top of each other.
VIEW_AZ, VIEW_EL = 20.0, 20.0

GREY_EVENT = '0.55'
GREY_REF = '0.45'
GREY_ROD = '0.45'
GREY_TEXT = '0.35'
FILL = '0.935'


# ── data ────────────────────────────────────────────────────────────────────────────

def load(run_dir):
    run = metrics.load_run(run_dir)
    d, ev = run['data'], run['events']
    log = open(os.path.join(run_dir, 'logs', 'launch.log')).read()
    hold = float(re.search(r'trajectory held ([0-9.]+) s for the reconfiguration', log).group(1))
    m = re.search(r'weld offset captured for slot (\d+): rho=\(([-0-9.]+),([-0-9.]+)\)', log)
    slot, rho = int(m.group(1)), (float(m.group(2)), float(m.group(3)))
    args = run['manifest']['launch_args']
    az = [float(a) for a in str(args['attach_azimuths_deg']).split(',')]
    par = open(os.path.join(run_dir, 'params', 'dissipative_controller.yaml')).read()
    r_ring = float(re.search(r'attach_radius:\s*([0-9.]+)', par).group(1))
    t = d['t']
    sweep = metrics.sweep_window(t, np.column_stack([d['payload_ref_x'], d['payload_ref_y']]))
    t_weld, t_land = ev['WELD'], ev['LAND']
    th = d['payload_tilt_deg']
    # level for good: the sample after the last one at or above LEVEL_DEG before LAND
    above = np.where((t >= t_weld) & (t <= t_land) & (th >= LEVEL_DEG))[0]
    return dict(d=d, t=t, t0=t[sweep][0], t_done=t[sweep][-1], t_attach=ev['MAGNET'],
                t_weld=t_weld, t_resume=t_weld + hold, t_land=t_land,
                t_level=t[above[-1] + 1], slot=slot, rho=rho, az=az, r_ring=r_ring,
                n=metrics.n_drones(d))


def rot(d, k):
    w, x, y, z = (d['payload_q' + a][k] for a in 'wxyz')
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pos(d, pre, k):
    return np.array([d[f'{pre}_x'][k], d[f'{pre}_y'][k], d[f'{pre}_z'][k]])


def rim(S, azimuth_deg):
    """Rim point at an azimuth in the load frame (0 deg = 3 o'clock, 90 deg = 12)."""
    a = np.radians(azimuth_deg)
    return S['r_ring'] * np.array([np.cos(a), np.sin(a), 0.0])


def anchors(S, k):
    """Load-frame rim anchor of every drone attached at sample k, {drone: (3,)}."""
    d, out = S['d'], {}
    for i in range(S['n']):
        if i < len(S['az']):
            out[i] = rim(S, S['az'][i])
        elif i == S['slot'] and d[f'd{i}_attached'][k] > 0.5:
            out[i] = np.array([S['rho'][0], S['rho'][1], 0.0])
    return out


# ── snapshot geometry ──────────────────────────────────────────────────────────────

def camera():
    a, e = np.radians(VIEW_AZ), np.radians(VIEW_EL)
    right = np.array([-np.sin(a), np.cos(a), 0.0])
    away = np.array([-np.cos(a), -np.sin(a), 0.0])
    up = np.array([0.0, 0.0, 1.0]) * np.cos(e) + away * np.sin(e)
    into = away * np.cos(e) - np.array([0.0, 0.0, 1.0]) * np.sin(e)
    return right, up, into


def project(p):
    """World offset (3,) or (N, 3) -> screen (u, v), orthographic."""
    right, up, _ = camera()
    p = np.atleast_2d(p)
    return np.column_stack([p @ right, p @ up])


def depth(p):
    return np.atleast_2d(p) @ camera()[2]


def snapshot(S, k):
    """Everything drawn in one frame, relative to the ring centre, in world axes."""
    d = S['d']
    P, Rk = pos(d, 'payload', k), rot(d, k)
    a = np.linspace(0, 2 * np.pi, 361)
    circle = S['r_ring'] * np.column_stack([np.cos(a), np.sin(a), 0 * a])
    ring = circle @ Rk.T
    anc = {i: Rk @ v for i, v in anchors(S, k).items()}
    drones = {i: pos(d, f'd{i}', k) - P for i in range(S['n'])}
    return dict(ring=ring, level=circle, anc=anc, drones=drones, R=Rk,
                tilt=float(d['payload_tilt_deg'][k]))


def split_near(pts):
    """The ring as two polylines, far half and near half (NaN-broken), for occlusion."""
    near = depth(pts) < 0
    uv = project(pts)
    far_uv, near_uv = uv.copy(), uv.copy()
    far_uv[near] = np.nan
    near_uv[~near] = np.nan
    return far_uv, near_uv


# ── figure ─────────────────────────────────────────────────────────────────────────

def tick_g(v, _):
    """Tick label without trailing zeros: 0, 0.5, 1."""
    return f'${v:g}$'


def at_or_after(t, te):
    """First sample at or after an event, so a frame at WELD already shows the weld."""
    return int(np.searchsorted(t, te))


def draw_frame(ax, S, sn, frame, lim, newcomer, clock):
    col = ps.PAYLOAD_LINE
    lev = project(sn['level'])
    ax.plot(lev[:, 0], lev[:, 1], ls=(0, (1, 1.6)), lw=0.7, color=GREY_REF, zorder=1)
    far, near = split_near(sn['ring'])
    ax.plot(far[:, 0], far[:, 1], lw=0.8, color=col, zorder=2)
    (u0, u1), (v0, v1) = lim
    for i, p in sn['drones'].items():
        uv = project(p)[0]
        if not (u0 <= uv[0] <= u1 and v0 <= uv[1] <= v1):
            continue                                   # the waiting newcomer, far off
        if i in sn['anc']:
            rod = project(np.vstack([sn['anc'][i], p]))
            ax.plot(rod[:, 0], rod[:, 1], lw=0.7, color=GREY_ROD, zorder=3)
        ax.plot(*uv, 'o', ms=5.6, mfc=ps.drone_colour(i), mec='white', mew=0.7, zorder=5)
        if i == newcomer and frame == 1:
            ax.annotate('newcomer', xy=uv, xytext=(-5, 0), textcoords='offset points',
                        ha='right', va='center', fontsize=8, color=ps.drone_colour(i))
    ax.plot(near[:, 0], near[:, 1], lw=1.5, color=col, zorder=4, solid_capstyle='round')
    for label, azd, off in clock or ():
        # clock position on the ring's own rim (load frame), beside that rim point
        uv = project(sn['R'] @ rim(S, azd))[0]
        ax.annotate(label, xy=uv, xytext=off, textcoords='offset points',
                    ha='center', va='center', fontsize=8, color=GREY_TEXT)
    ax.set_xlim(u0, u1)
    ax.set_ylim(v0, v1)
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run', nargs='?', default=RUN)
    ap.add_argument('--out', default=OUT)
    a = ap.parse_args()
    S = load(a.run)
    d, t = S['d'], S['t']
    newcomer = S['slot']
    c3 = ps.drone_colour(newcomer)
    win = (t >= S['t0']) & (t <= S['t_land'])

    # ── snapshot data, and one window for all three frames
    names = [('attach', S['t_attach']), ('weld', S['t_weld']), ('resume', S['t_resume'])]
    snaps = [snapshot(S, at_or_after(t, te)) for _, te in names]
    pts = []
    for j, sn in enumerate(snaps):
        pts += [project(sn['ring']), project(sn['level'])]
        pts += [project(p) for i, p in sn['drones'].items() if i in sn['anc']]
    pts = np.vstack(pts)
    lo, hi = pts.min(0), pts.max(0)
    mu, mv = 0.10 * (hi[0] - lo[0]), 0.10 * (hi[1] - lo[1])
    lim = ((lo[0] - mu, hi[0] + mu), (lo[1] - mv, hi[1] + mv))
    frame_aspect = (lim[1][1] - lim[1][0]) / (lim[0][1] - lim[0][0])

    # ── plan data and crop: the ring path and the ring at the weld; the newcomer is
    # cropped where it leaves that box (it starts well south of everything else)
    px, py = d['payload_x'][win], d['payload_y'][win]
    kw = at_or_after(t, S['t_weld'])
    Pw, Rw = pos(d, 'payload', kw), rot(d, kw)
    ang = np.linspace(0, 2 * np.pi, 361)
    rw = (S['r_ring'] * np.column_stack([np.cos(ang), np.sin(ang), 0 * ang])) @ Rw.T + Pw
    xs = np.concatenate([px, rw[:, 0]])
    ys = np.concatenate([py, rw[:, 1]])
    span = max(np.nanmax(xs) - np.nanmin(xs), np.nanmax(ys) - np.nanmin(ys))
    pad = 0.05 * span
    xlim = (np.nanmin(xs) - pad, np.nanmax(xs) + pad)
    # a fifth of the span below the ring path shows where the newcomer comes from
    ylim = (np.nanmin(ys) - 0.25 * span, np.nanmax(ys) + pad)
    plan_aspect = (ylim[1] - ylim[0]) / (xlim[1] - xlim[0])

    ps.thesis()
    W, H = ps.size('full', height=4.1)
    fig = plt.figure(figsize=(W, H))
    gs = fig.add_gridspec(2, 1, height_ratios=[1.0, 1.9])
    gsf = gs[0].subgridspec(1, 3, wspace=0.0)
    ax_f = [fig.add_subplot(gsf[0, j]) for j in range(3)]
    gsb = gs[1].subgridspec(1, 2, width_ratios=[1.0, 2.6])
    ax_p = fig.add_subplot(gsb[0, 0])
    ax_t = fig.add_subplot(gsb[0, 1])

    # (a) snapshots ------------------------------------------------------------------
    clocks = [[('6', 270.0, (-6, -4)), ('12', 90.0, (9, -3))], None, None]
    for j, (ax, (nm, te), sn) in enumerate(zip(ax_f, names, snaps)):
        draw_frame(ax, S, sn, j, lim, newcomer, clocks[j])
        ax.text(0.5, 1.0, rf'\textsc{{{nm}}}\enspace ${te:.0f}$\,s', transform=ax.transAxes,
                ha='center', va='bottom', fontsize=8)
        ax.text(0.5, 0.0, rf'$\theta = {sn["tilt"]:.0f}^\circ$', transform=ax.transAxes,
                ha='center', va='top', fontsize=8)
    # scale bar, once: horizontal on the page is true length in this projection
    bar = S['r_ring']
    (u0, u1), (v0, v1) = lim
    ub, vb = u1 - 0.06 * (u1 - u0) - bar, v0 + 0.06 * (v1 - v0)
    ax_f[0].plot([ub, ub + bar], [vb, vb], lw=1.0, color='0.2', solid_capstyle='butt')
    ax_f[0].text(ub + bar / 2, vb, rf'${bar:g}$\,m', ha='center', va='bottom',
                 fontsize=8, color='0.2')

    # (b) plan view ------------------------------------------------------------------
    ax_p.plot(px, py, lw=1.0, color=ps.PAYLOAD_LINE, zorder=3, rasterized=px.size > 2000)
    ax_p.plot(rw[:, 0], rw[:, 1], lw=0.6, color=ps.PAYLOAD_LINE, zorder=2)
    for i, v in anchors(S, kw).items():
        p = Rw @ v + Pw
        ax_p.plot(p[0], p[1], 'o', ms=4.0, mfc=ps.drone_colour(i), mec='white', mew=0.5,
                  zorder=5)
    app = (t >= S['t_attach']) & (t <= S['t_weld'])
    nx, ny = d[f'd{newcomer}_x'][app], d[f'd{newcomer}_y'][app]
    ax_p.plot(nx, ny, lw=1.0, color=c3, zorder=4)
    # direction of the approach, where it enters the crop
    ke = int(np.argmax(ny > ylim[0] + 0.03 * (ylim[1] - ylim[0])))
    ax_p.annotate('', xy=(nx[ke + 3], ny[ke + 3]), xytext=(nx[ke], ny[ke]),
                  arrowprops=dict(arrowstyle='-|>', mutation_scale=9, lw=0, color=c3,
                                  shrinkA=0, shrinkB=0), zorder=4)
    ax_p.annotate('newcomer', xy=(nx[ke], ny[ke]), xytext=(-6, 2), ha='right',
                  va='center', fontsize=8, color=c3, textcoords='offset points')
    ax_p.annotate(r'\textsc{weld}', xy=(nx[-1], ny[-1]), xytext=(-14, -12), ha='right',
                  va='center', fontsize=8, textcoords='offset points',
                  arrowprops=dict(arrowstyle='-', lw=0.5, color='0.4', shrinkA=1,
                                  shrinkB=2.5))
    for name, te, off, ha in (('attach', S['t_attach'], (-16, 14), 'right'),
                              ('resume', S['t_resume'], (26, -20), 'left')):
        k = at_or_after(t, te)
        xy = (d['payload_x'][k], d['payload_y'][k])
        ax_p.plot(*xy, 'o', ms=3.0, mfc='0.1', mec='white', mew=0.5, zorder=6)
        ax_p.annotate(rf'\textsc{{{name}}}', xy=xy, xytext=off, ha=ha, va='center',
                      fontsize=8, textcoords='offset points',
                      arrowprops=dict(arrowstyle='-', lw=0.5, color='0.4', shrinkA=1,
                                      shrinkB=2.5))
    ax_p.plot(px[0], py[0], 'o', ms=3.4, mfc='white', mec='0.15', mew=0.8, zorder=6)
    ax_p.annotate('start', xy=(px[0], py[0]), xytext=(0, 4), ha='center', va='bottom',
                  fontsize=8, textcoords='offset points')
    kr = np.where(t[win] > S['t_resume'])[0]
    k1 = kr[len(kr) // 6]
    ax_p.annotate('', xy=(px[k1 + 6], py[k1 + 6]), xytext=(px[k1], py[k1]),
                  arrowprops=dict(arrowstyle='-|>', mutation_scale=10, lw=0,
                                  color=ps.PAYLOAD_LINE, shrinkA=0, shrinkB=0), zorder=4)
    ktop = int(np.nanargmax(py))
    ax_p.annotate('ring', xy=(px[ktop], py[ktop]), xytext=(0, 3), ha='center',
                  va='bottom', fontsize=8, color=ps.PAYLOAD_LINE, textcoords='offset points')
    ax_p.set_xlim(*xlim)
    ax_p.set_ylim(*ylim)
    ax_p.set_aspect('equal', adjustable='box')
    ax_p.set_xlabel(r'Position $x$ (m)')
    ax_p.set_ylabel(r'Position $y$ (m)')
    for axis in (ax_p.xaxis, ax_p.yaxis):
        axis.set_major_locator(plt.MultipleLocator(0.5))
        axis.set_major_formatter(plt.FuncFormatter(tick_g))

    # (c) tilt -------------------------------------------------------------------------
    tt, th = t[win], d['payload_tilt_deg'][win]
    ax_t.axvspan(S['t_attach'], S['t_resume'], color=FILL, lw=0, zorder=0)
    events = [('attach', S['t_attach']), ('weld', S['t_weld']), ('resume', S['t_resume']),
              ('circle done', S['t_done'])]
    for name, te in events:
        ax_t.axvline(te, color=GREY_EVENT, lw=0.5, zorder=1)
        ax_t.annotate(rf'\textsc{{{name}}}', xy=(te, 1.0), xycoords=('data', 'axes fraction'),
                      xytext=(0, 3), textcoords='offset points', ha='center', va='bottom',
                      fontsize=8)
    ax_t.axhline(LEVEL_DEG, ls=(0, (3.5, 2)), lw=0.8, color=GREY_REF, zorder=2)
    ax_t.plot(tt, th, lw=0.85, color=ps.PAYLOAD_LINE, zorder=3, rasterized=tt.size > 2000)
    # WELD -> under the 10 deg line for good, drawn just above that line
    yb = LEVEL_DEG + 3.0
    ax_t.annotate('', xy=(S['t_level'], yb), xytext=(S['t_weld'], yb),
                  arrowprops=dict(arrowstyle='<|-|>', mutation_scale=5, lw=0.5,
                                  color='0.2', shrinkA=0, shrinkB=0), zorder=4)
    ax_t.text(S['t_weld'] + 0.5, yb + 1.0, rf'${S["t_level"] - S["t_weld"]:.1f}$\,s',
              ha='left', va='bottom', fontsize=8)
    ax_t.set_xlim(S['t0'], S['t_land'])
    ax_t.set_ylim(0, 45)
    ax_t.yaxis.set_major_locator(plt.MultipleLocator(10))
    ax_t.xaxis.set_major_locator(plt.MultipleLocator(10))
    ax_t.set_xlabel(r'Time $t$ (s)')
    ax_t.set_ylabel(r'Ring tilt $\theta$ (\si{\degree})')
    seg3 = (tt >= S['t_attach']) & (tt <= S['t_weld'])
    seg4 = tt >= S['t_resume']
    mid3 = 0.5 * (S['t_attach'] + S['t_weld'])
    ax_t.text(mid3, np.nanmax(th[seg3]) + 1.5, '3 drones', ha='center', va='bottom',
              fontsize=8)
    ax_t.text(0.5 * (S['t_resume'] + S['t_done']), np.nanmax(th[seg4]) + 1.5, '4 drones',
              ha='center', va='bottom', fontsize=8)
    ax_t.text(S['t_weld'], 1.2, 'target held', ha='center', va='bottom', fontsize=8,
              color=GREY_TEXT, zorder=2,
              bbox=dict(boxstyle='square,pad=0.15', fc=FILL, ec='none'))

    # layout: size the plan's column to its equal-aspect box, and the snapshot row to
    # its frames, then put the panel labels at the top-left of each panel's ink
    fig.canvas.draw()
    rh, rw_ = [1.0, 1.9], [1.0, 2.6]
    for _ in range(4):
        bp = ax_p.get_position(original=True)
        need_w = bp.height * H / plan_aspect / W
        rw_[0] *= need_w / bp.width
        gsb.set_width_ratios(rw_)
        bf = ax_f[0].get_position(original=True)
        need_h = bf.width * W * frame_aspect / H
        rh[0] *= need_h / bf.height
        gs.set_height_ratios(rh)
        fig.canvas.draw()
    fig.set_constrained_layout(False)
    r = fig.canvas.get_renderer()
    inv = fig.transFigure.inverted()

    def ink(ax):
        return inv.transform(ax.get_tightbbox(r).get_points())

    top_bottom_row = max(ink(ax_t)[1, 1], ink(ax_p)[1, 1])
    for ax, letter, y in ((ax_f[0], 'a', ink(ax_f[0])[1, 1]),
                          (ax_p, 'b', top_bottom_row), (ax_t, 'c', top_bottom_row)):
        box = ax.get_position()
        x_ink = min(ink(ax)[0, 0], box.x0)
        dx = (x_ink - box.x0) * W * 72
        dy = (y - box.y1) * H * 72 - 9
        ps.panel_label(ax, letter, dx_pt=dx, dy_pt=dy)
    paths = ps.save_print(fig, a.out)
    print('\n'.join(paths))
    print(f"t0={S['t0']:.2f} done={S['t_done']:.2f} level={S['t_level']:.2f} "
          f"after weld {S['t_level'] - S['t_weld']:.2f} s")


if __name__ == '__main__':
    main()
