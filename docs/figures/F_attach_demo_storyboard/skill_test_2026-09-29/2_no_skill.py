#!/usr/bin/env python3
"""
Attach during a circle (Gazebo run R0209) as one thesis figure.

    python3 storyboard.py            # -> storyboard.pdf / storyboard.png next to this file

(a) four snapshots of the fleet drawn from the logged poses (ring pose from the payload
    quaternion, rods from the rim attach points to the logged drone positions), one
    fixed oblique camera, one scale;
(b) plan view: payload target, payload, and the newcomer's approach to the 6 o'clock rim;
(c) ring tilt against time with the target-held interval and the events.

Everything drawn comes from the run directory: run.csv, events.csv, the hold length from
launch.log, ring radius and attach azimuths from the run's manifest and params.json.
"""
import csv
import glob
import json
import os
import re

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                                        # noqa: E402
from matplotlib.patches import FancyArrowPatch                        # noqa: E402
from matplotlib.ticker import FormatStrFormatter                      # noqa: E402

RUN = '/home/wesley/multi_drone_control/results/2026-09-10/R0209_sim_gz_attach_circle_n3_fast'
OUT = os.path.dirname(os.path.abspath(__file__))
RUN_ID = 'R0209'

# thesis/main.tex: article 12 pt, A4, 1 in margins -> \textwidth = 452.97 pt
TEXTWIDTH = 452.97 / 72.27

INK = '#1a1a1a'
MUTED = '#6b6b6b'
FAINT = '#b5b5b5'
PAYLOAD = '#b8860b'          # the thesis payload colour (plot_style.PAYLOAD_LINE)
NEWCOMER = '#d62728'         # drone 3 in RViz and in every other thesis figure
CARRIER = '#4a4a4a'
HELD = '#efefef'

SNAPSHOTS = (15.0, None, None, 50.0)   # WELD and RESUME are filled in from the log
CAM_AZ, CAM_EL = -30.0, 22.0


# ── data ──────────────────────────────────────────────────────────────────────

def load():
    rows = list(csv.DictReader(open(os.path.join(RUN, 'logs', 'run.csv'))))

    def col(k):
        return np.array([float(r[k]) if r.get(k) not in (None, '', 'nan') else np.nan
                         for r in rows])

    d = {k: col(k) for k in rows[0] if k != 'step'}
    ev = {}
    for line in open(os.path.join(RUN, 'logs', 'events.csv')):
        p = line.strip().split(',')
        if p and p[0] and p[0] != 'sim_time':
            ev.setdefault(p[1], float(p[0]))
    hold = None
    for line in open(os.path.join(RUN, 'logs', 'launch.log')):
        m = re.search(r'trajectory held ([0-9.]+) s for the reconfiguration', line)
        if m:
            hold = float(m.group(1))
            break
    la = json.load(open(os.path.join(RUN, 'manifest.json')))['launch_args']
    params = json.load(open(glob.glob(os.path.join(
        RUN, 'logs', 'controller_quad_load', 'dissipative_controller_*', 'params.json'))[0]))
    geom = dict(
        r_ring=float(params['attach_radius']),
        az=[float(a) for a in str(la['attach_azimuths_deg']).split(',')],
        az_new=float(np.degrees(np.arctan2(la['attach_y_offset'], la['attach_x_offset']))) % 360,
        n_carriers=int(la['num_drones']),
    )
    return d, ev, hold, geom


def rotation(qw, qx, qy, qz):
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
        [2 * (qx * qy + qw * qz), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qw * qx)],
        [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx * qx + qy * qy)]])


def idx(d, t):
    return int(np.nanargmin(np.abs(d['t'] - t)))


# ── style ─────────────────────────────────────────────────────────────────────

def style():
    plt.rcParams.update(plt.rcParamsDefault)
    plt.rcParams.update({
        'text.usetex': True,
        'text.latex.preamble': r'\usepackage{amsmath}',
        'font.family': 'serif',
        'font.size': 8.5,
        'axes.labelsize': 8.5,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'axes.linewidth': 0.6,
        'axes.edgecolor': INK,
        'axes.labelcolor': INK,
        'xtick.color': INK,
        'ytick.color': INK,
        'axes.spines.top': False,
        'axes.spines.right': False,
        'xtick.direction': 'out',
        'ytick.direction': 'out',
        'xtick.major.size': 2.8,
        'ytick.major.size': 2.8,
        'xtick.major.width': 0.6,
        'ytick.major.width': 0.6,
        'xtick.major.pad': 2,
        'ytick.major.pad': 2,
        'axes.labelpad': 2.5,
        'lines.solid_capstyle': 'round',
        'lines.dash_capstyle': 'round',
        'savefig.dpi': 300,
        'pdf.fonttype': 42,
    })


def tag(ax, x, y, n, coords='data', dx=0, dy=0, size=6.5):
    """The small numbered disc that ties a snapshot in (a) to a place in (b) and (c)."""
    ax.annotate(rf'\textbf{{{n}}}', (x, y), xycoords=coords, xytext=(dx, dy),
                textcoords='offset points', ha='center', va='center', fontsize=size,
                color='white', zorder=20,
                bbox=dict(boxstyle='circle,pad=0.18', fc=INK, ec='none'))


def axes_in(fig, x, y, w, h):
    W, H = fig.get_size_inches()
    return fig.add_axes([x / W, y / H, w / W, h / H])


# ── (a) snapshots ─────────────────────────────────────────────────────────────

class Camera:
    """Orthographic camera at azimuth `az` and elevation `el` (deg), looking at the ring."""

    def __init__(self, az, el):
        a, e = np.radians(az), np.radians(el)
        self.c = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        self.r = np.array([-np.sin(a), np.cos(a), 0.0])
        self.u = np.cross(self.c, self.r)

    def __call__(self, p):
        p = np.atleast_2d(p)
        return p @ self.r, p @ self.u, p @ self.c      # screen x, screen y, depth


def scene(d, geom, t):
    """World geometry at time t, relative to the ring centre."""
    i = idx(d, t)
    p = np.array([d['payload_x'][i], d['payload_y'][i], d['payload_z'][i]])
    R = rotation(d['payload_qw'][i], d['payload_qx'][i], d['payload_qy'][i], d['payload_qz'][i])
    th = np.linspace(0, 2 * np.pi, 241)
    rr = geom['r_ring']
    ring = (R @ np.vstack([rr * np.cos(th), rr * np.sin(th), 0 * th])).T
    level = np.vstack([rr * np.cos(th), rr * np.sin(th), 0 * th]).T
    drones = []
    azs = geom['az'] + [geom['az_new']]
    for k, az in enumerate(azs):
        attached = k < geom['n_carriers'] or d[f'd{k}_attached'][i] > 0.5
        dk = np.array([d[f'd{k}_x'][i], d[f'd{k}_y'][i], d[f'd{k}_z'][i]]) - p
        a = R @ np.array([rr * np.cos(np.radians(az)), rr * np.sin(np.radians(az)), 0.0])
        new = k >= geom['n_carriers']
        drones.append(dict(k=k, pos=dk, anchor=a, attached=attached, newcomer=new,
                           colour=NEWCOMER if new else CARRIER))
    return dict(i=i, t=d['t'][i], p=p, ring=ring, level=level, drones=drones,
                tilt=d['payload_tilt_deg'][i])


def draw_scene(ax, sc, cam, trail=None):
    # the level ring through the same centre: the gap between it and the ring is the tilt
    lx, ly, _ = cam(sc['level'])
    ax.plot(lx, ly, color=FAINT, lw=0.6, ls=(0, (1, 1.6)), zorder=1)

    x, y, z = cam(sc['ring'])
    back = np.where(z < 0, y, np.nan)
    front = np.where(z >= 0, y, np.nan)
    ax.plot(x, back, color=PAYLOAD, lw=1.3, alpha=0.45, zorder=2)
    ax.plot(x, front, color=PAYLOAD, lw=1.7, zorder=6)

    if trail is not None:
        tx, ty, _ = cam(trail)
        ax.plot(tx, ty, color=NEWCOMER, lw=0.7, ls=(0, (1, 1.5)), alpha=0.9, zorder=3)

    arm = 0.075
    for dr in sc['drones']:
        px, py, pz = (v[0] for v in cam(dr['pos']))
        if dr['attached']:
            ax_, ay_, az_ = (v[0] for v in cam(dr['anchor']))
            zo = 4 if (az_ + pz) < 0 else 7
            ax.plot([ax_, px], [ay_, py], color=dr['colour'], lw=0.75, zorder=zo)
            ax.plot(ax_, ay_, 'o', ms=2.6, mfc=dr['colour'], mec='white', mew=0.4, zorder=zo + 0.5)
        elif dr['newcomer']:
            # the reserved slot the newcomer will take, still empty
            ax_, ay_, az_ = (v[0] for v in cam(dr['anchor']))
            ax.plot(ax_, ay_, 'o', ms=3.0, mfc='white', mec=dr['colour'], mew=0.7, zorder=7)
            continue
        zo = 5 if pz < 0 else 8
        # quadrotor seen from the side: arms as a short bar, hub as a dot
        ax.plot([px - arm, px + arm], [py, py], color=dr['colour'], lw=1.4,
                solid_capstyle='butt', zorder=zo)
        ax.plot(px, py, 'o', ms=2.8, mfc='white', mec=dr['colour'], mew=0.8, zorder=zo + 0.1)


def extents(scenes, cam):
    pts = []
    for sc in scenes:
        x, y, _ = cam(sc['ring'])
        pts += list(zip(x, y))
        for dr in sc['drones']:
            if dr['attached']:
                px, py, _ = cam(dr['pos'])
                pts.append((px[0], py[0]))
    pts = np.array(pts)
    return pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max()


# ── figure ────────────────────────────────────────────────────────────────────

def main():
    d, ev, hold, geom = load()
    style()
    t = d['t']
    t_to, t_att, t_weld, t_land = ev['TAKEOFF'], ev['MAGNET'], ev['WELD'], ev['LAND']
    t_res = t_weld + hold
    nw = f"d{geom['n_carriers']}_"                                     # the newcomer's columns
    t_weld_row = t[np.argmax((t >= t_weld) & (d[nw + 'attached'] > 0.5))]
    snaps = [SNAPSHOTS[0], t_weld_row, t_res, SNAPSHOTS[3]]

    W, H = TEXTWIDTH, 4.15
    fig = plt.figure(figsize=(W, H))

    # (a) ------------------------------------------------------------------------
    cam = Camera(az=CAM_AZ, el=CAM_EL)
    scenes = [scene(d, geom, ts) for ts in snaps]
    # the newcomer's last seconds before the weld, to show where it came from
    tr = (t >= t_weld - 5.5) & (t <= t_weld)
    trail = np.vstack([d[nw + 'x'][tr], d[nw + 'y'][tr], d[nw + 'z'][tr]]).T - scenes[1]['p']

    x0, x1, y0, y1 = extents(scenes, cam)
    pad = 0.06
    x0, x1, y0, y1 = x0 - pad, x1 + pad, y0 - pad, y1 + pad
    top_h = 1.30
    fw = W / 4
    scale = min(fw / (x1 - x0), (top_h - 0.34) / (y1 - y0))     # inches per metre
    ybase = H - top_h - 0.02
    labels = ('three drones on the circle', r"\textsc{weld} at the 6 o'clock rim",
              r'\textsc{resume}, ring levelled', 'four drones on the circle')
    for j, (sc, lab) in enumerate(zip(scenes, labels)):
        ax = axes_in(fig, j * fw, ybase, fw, top_h)
        ax.set_axis_off()
        cx = 0.5 * (x0 + x1)
        half_w = 0.5 * fw / scale
        # scene sits between a header line and a footer line
        ax.set_xlim(cx - half_w, cx + half_w)
        ylo = y0 - 0.20 / scale
        ax.set_ylim(ylo, ylo + top_h / scale)
        draw_scene(ax, sc, cam, trail=trail if j == 1 else None)
        if j == 1:
            tx, ty, _ = cam(trail)
            k = int(np.argmin(tx))
            ax.annotate('newcomer', (tx[k], ty[k]), xytext=(1, -5), textcoords='offset points',
                        ha='left', va='top', fontsize=6.5, color=NEWCOMER)
        hx = 0.36 / fw
        tag(ax, hx, 0.92, j + 1, coords='axes fraction')
        ax.annotate(rf'$t = {sc["t"]:.0f}$\,s', (hx, 0.92), xycoords='axes fraction',
                    xytext=(7, 0), textcoords='offset points', ha='left', va='center',
                    fontsize=8.5, color=INK)
        ax.text(0.5, 0.02, lab, transform=ax.transAxes, ha='center', va='bottom',
                fontsize=7.5, color=MUTED)
        # tilt, written beside the low side of the ring
        rx, ry, _ = cam(sc['ring'])
        k = int(np.argmin(ry))
        ax.annotate(rf'${sc["tilt"]:.0f}^\circ$', (rx[k], ry[k]), xytext=(-7, -2.5),
                    textcoords='offset points', ha='right', va='top', fontsize=7.5,
                    color=PAYLOAD)
    fig.text(0.004, (ybase + 0.92 * top_h) / H, '(a)', ha='left', va='center', fontsize=9)

    # 0.25 m scale bar under the first frame, and a name for the dotted ellipse
    ax0 = fig.axes[0]
    lx, ly, _ = cam(scenes[0]['level'])
    k = int(np.argmax(lx))
    ax0.annotate('level', (lx[k], ly[k]), xytext=(2, -1), textcoords='offset points',
                 ha='left', va='top', fontsize=6.5, color=MUTED)
    xb = x0 + 0.02
    yb = ax0.get_ylim()[0] + 0.30 / scale
    ax0.plot([xb, xb + 0.25], [yb, yb], color=INK, lw=0.8, solid_capstyle='butt')
    ax0.text(xb + 0.125, yb + 0.02, r'0.25\,m', ha='center', va='bottom', fontsize=6.5,
             color=INK)

    # (b) plan view ----------------------------------------------------------------
    bot, bh = 0.40, H - top_h - 0.02 - 0.40 - 0.30
    xl, yl = (-0.50, 0.78), (-0.48, 1.20)
    bw = bh * (xl[1] - xl[0]) / (yl[1] - yl[0])
    axb = axes_in(fig, 0.46, bot, bw, bh)
    fly = (t >= t_to) & (t <= t_land)
    # at the weld the held target is re-anchored to where the ring is: a jump, not a path
    rx, ry = d['payload_ref_x'][fly].copy(), d['payload_ref_y'][fly].copy()
    jump = np.r_[False, np.hypot(np.diff(rx), np.diff(ry)) > 0.05]
    rx[jump], ry[jump] = np.nan, np.nan
    axb.plot(rx, ry, color=MUTED, lw=0.8, ls=(0, (3.5, 2.2)), zorder=2)
    axb.plot(d['payload_x'][fly], d['payload_y'][fly], color=PAYLOAD, lw=1.2, zorder=3)
    app = (t >= t_att) & (t <= t_weld)
    axb.plot(d[nw + 'x'][app], d[nw + 'y'][app], color=NEWCOMER, lw=0.9, zorder=4)
    # arrowhead on the newcomer's approach
    ka = np.where(app)[0]
    k1 = ka[int(0.62 * len(ka))]
    axb.add_patch(FancyArrowPatch((d[nw + 'x'][k1 - 8], d[nw + 'y'][k1 - 8]), (d[nw + 'x'][k1], d[nw + 'y'][k1]),
                                  arrowstyle='-|>,head_length=3.2,head_width=1.8', lw=0,
                                  color=NEWCOMER, zorder=4, mutation_scale=1))
    # ring and its four anchors at the weld, in plan
    sw = scenes[1]
    axb.plot(sw['p'][0] + sw['ring'][:, 0], sw['p'][1] + sw['ring'][:, 1], color=PAYLOAD,
             lw=0.6, alpha=0.8, zorder=3)
    for dr in sw['drones']:
        a = sw['p'] + dr['anchor']
        axb.plot(a[0], a[1], 'o', ms=2.6, mfc=dr['colour'], mec='white', mew=0.4, zorder=5)
    # frozen target
    kh = idx(d, t_att + 0.5)
    axb.plot(d['payload_ref_x'][kh], d['payload_ref_y'][kh], 's', ms=3.2, mfc='white',
             mec=INK, mew=0.7, zorder=6)
    axb.annotate(r'target held', (d['payload_ref_x'][kh], d['payload_ref_y'][kh]),
                 xytext=(0.08, 0.54), textcoords='data', fontsize=7.5, color=INK,
                 ha='center', va='bottom',
                 arrowprops=dict(arrowstyle='-', lw=0.5, color=INK, shrinkA=1, shrinkB=2.5))
    # direction of travel on the circle
    kd = idx(d, 45.0)
    axb.add_patch(FancyArrowPatch((d['payload_ref_x'][kd - 3], d['payload_ref_y'][kd - 3]),
                                  (d['payload_ref_x'][kd], d['payload_ref_y'][kd]),
                                  arrowstyle='-|>,head_length=3.2,head_width=1.8', lw=0,
                                  color=MUTED, zorder=2, mutation_scale=1))
    for j, ts in enumerate(snaps):
        k = idx(d, ts)
        axb.plot(d['payload_x'][k], d['payload_y'][k], 'o', ms=2.2, color=INK, zorder=7)
    for j, off in enumerate([(-8, 6), (-9, 5), (9, -3), (-8, 0)]):
        k = idx(d, snaps[j])
        tag(axb, d['payload_x'][k], d['payload_y'][k], j + 1, dx=off[0], dy=off[1])
    axb.text(-0.24, 1.07, 'target', fontsize=7.5, color=MUTED, ha='right', va='bottom')
    axb.text(0.45, 1.05, 'payload', fontsize=7.5, color=PAYLOAD, ha='left', va='bottom')
    axb.text(0.40, -0.40, 'newcomer', fontsize=7.5, color=NEWCOMER, ha='left', va='bottom')
    axb.set_xlim(*xl)
    axb.set_ylim(*yl)
    axb.set_aspect('equal', adjustable='box')
    axb.set_xticks([-0.4, 0, 0.4])
    axb.set_yticks([0, 0.5, 1.0])
    for a_ in (axb.xaxis, axb.yaxis):
        a_.set_major_formatter(FormatStrFormatter('$%g$'))
    axb.set_xlabel(r'$x$ (m)')
    axb.set_ylabel(r'$y$ (m)')
    fig.text(0.004, (bot + bh + 0.20) / H, '(b)', ha='left', va='top', fontsize=9)

    # (c) tilt ------------------------------------------------------------------------
    cx0 = 0.46 + bw + 0.62
    axc = axes_in(fig, cx0, bot, W - cx0 - 0.13, bh)
    tl = (t >= t_to) & (t <= t_land + 2)
    axc.axvspan(t_att, t_res, color=HELD, lw=0, zorder=0)
    axc.axhline(10, color=MUTED, lw=0.5, ls=(0, (3, 2)), zorder=1)
    axc.plot(t[tl], d['payload_tilt_deg'][tl], color=PAYLOAD, lw=0.6, zorder=3)
    for name, te in ((r'\textsc{attach}', t_att), (r'\textsc{weld}', t_weld),
                     (r'\textsc{resume}', t_res), (r'\textsc{land}', t_land)):
        axc.axvline(te, color=MUTED, lw=0.5, zorder=2)
        axc.text(te, 1.02, name, transform=axc.get_xaxis_transform(), ha='center',
                 va='bottom', fontsize=7.5, color=INK)
    axc.text(t_att + 7.5, 21, 'target held', ha='center', va='center', fontsize=7.5,
             color=MUTED, zorder=2.5,
             bbox=dict(boxstyle='square,pad=0.15', fc=HELD, ec='none'))
    axc.text(t_land - 0.8, 10.8, r'$10^\circ$', ha='right', va='bottom', fontsize=7.5,
             color=MUTED)
    axc.text(13, 40.3, '3 drones', ha='center', va='bottom', fontsize=7.5, color=INK)
    axc.text(53, 12.5, '4 drones', ha='center', va='bottom', fontsize=7.5, color=INK)
    axc.set_xlim(t_to, t_land + 2)
    axc.set_ylim(0, 46)
    axc.set_yticks([0, 10, 20, 30, 40])
    axc.set_xlabel(r'$t$ (s)')
    axc.set_ylabel(r'ring tilt (deg)')
    fig.text((cx0 - 0.50) / W, (bot + bh + 0.20) / H, '(c)', ha='left', va='top', fontsize=9)

    fig.savefig(os.path.join(OUT, 'storyboard.pdf'))
    fig.savefig(os.path.join(OUT, 'storyboard.png'), dpi=300)
    print('wrote', os.path.join(OUT, 'storyboard.pdf'))


if __name__ == '__main__':
    main()
