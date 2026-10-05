#!/usr/bin/env python3
"""
tools/attach_storyboard.py -- attach during a circle, as one thesis figure.

    python3 tools/attach_storyboard.py R0209 [--out DIR]
    -> DIR/attach_storyboard_R0209.pdf (for LaTeX) and .png (for looking)
       DIR defaults to docs/figures/F_attach_demo_storyboard

(a) four snapshots of the fleet rebuilt from the logged poses, one orthographic camera
    and one scale; (b) plan view; (c) ring tilt against time.

Brief (thesis-figure skill, step 1)
-----------------------------------
1. Sentence. Three drones carry the ring tilted by about 35 deg; the newcomer welds to
   the ring at its empty 6 o'clock rim; during the 8 s hand-out the tilt first peaks,
   then falls, and from 9.3 s after the weld it stays under 10 deg (mean 4.4 deg, a
   1.5 Hz oscillation); the four drones fly the rest of the circle, 0.20 m RMS from the
   target against 0.02 m for three.
2. Hero. The storyboard: (a) across the top carries "tilted, newcomer at 6 o'clock,
   level, four on the circle"; (c), the widest plot, carries the numbers. (b) supports
   both: where each snapshot happened, where the target was held, where the newcomer
   came from, and how far the ring ran from the target.
3. Left out. Carrier identities (grey: the claim does not need them), carrier paths,
   heights, tensions, the takeoff transient, the hover after the circle and the LAND
   (58-68 s), the newcomer before ATTACH (below the crop of (b)), the newcomer's
   approach trail in (a) ((b) draws it), a legend, snapshot subtitles, a fifth snapshot
   at the 43 deg peak ((c) marks it; a fifth frame would shrink the other four), a
   ring-to-target distance strip (its three numbers are in the caption).
4. Window. (c) and the paths in (b): the target's sweep (metrics.sweep_window), from
   where the target starts moving to where the circle ends. Snapshots 1 and 4 sit
   half-way through the three-drone and the four-drone parts of the sweep, 2 on the
   first sample with the newcomer welded, 3 at RESUME.
5. Caption: printed by this script with the run's numbers (no run id).
6. Evidence map (caption clause -> mark).
   - 0.6 kg, radius 0.25 m, 0.5 m rods -> manifest/params (printed); rods to scale
     against the 0.25 m bar in (a).
   - three drones at 3, 12 and 9 o'clock -> (a)1: clock labels on the carriers' rim
     anchors, the empty slot labelled "6 o'clock", the x-y triad; (b): the grey rim
     anchors at +x, +y and -x of the ring at the weld.
   - tilted by about 35 deg -> (a)1, (a)2: tilt printed against the dotted level ring;
     (c): the trace before WELD under the "3 drones" bracket, badges 1 and 2 on it.
   - ATTACH holds the target; the ring drifts from it -> (c): grey band from ATTACH,
     "target held"; (b): the x where the dashed target stops, and the ring trace from
     there to badge 2.
   - welds at the 6 o'clock rim -> (a)2: the red newcomer (filled hub) on the 6 o'clock
     rim; (b): the red approach ends on the red 6 o'clock anchor.
   - hand-out over 8 s -> (c): the "hand-out" bracket from WELD.
   - peaks at 43 deg -> (c): the peak labelled.
   - under 10 deg from 9.3 s after the weld, peak after that, mean -> (c): dashed 10 deg
     reference, the WELD -> 9.3 s arrow ending on an open circle at the last crossing,
     the mean line under the "4 drones" bracket; (a)3, (a)4: tilts.
   - drifts 0.16 m during the hand-out, flies the rest of the circle -> (b): badges 2
     and 3; the gold trace against the dashed target, start and end marked.

Every number drawn comes from the run directory: logs/run.csv (poses, tilt, target,
attach flag), logs/events.csv (ATTACH = MAGNET, WELD), logs/launch.log (the
reconfiguration hold, hence RESUME; the weld offset on the rim and the newcomer's slot),
manifest.json (carrier rim azimuths, mass, rod length, hand-out time) and
params/dissipative_planner.yaml (dissipative_controller.yaml before the rename)
(rim radius). The only typed number is the 10 deg of the claim itself.
"""
import argparse
import os
import re
import sys

import numpy as np

sys.dont_write_bytecode = True            # reading the repo must not write into it
TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import plot_style as ps                                            # noqa: E402
import metrics                                                     # noqa: E402
from run_dir import resolve                                        # noqa: E402
import matplotlib.pyplot as plt                                    # noqa: E402
from matplotlib.patches import FancyArrowPatch                     # noqa: E402

REPO = os.path.dirname(TOOLS)
OUT = os.path.join(REPO, 'docs', 'figures', 'F_attach_demo_storyboard')

LEVEL_DEG = 10.0                    # the claim: "under 10 deg"

# One orthographic camera for every snapshot: azimuth of the viewer about z (from +x)
# and elevation (deg). From the south-west the ring's tilt reads as a see-saw, the
# empty 6 o'clock rim faces the viewer, and no drone stands on the ring or on another
# drone's rod in any of the four frames.
VIEW_AZ, VIEW_EL = -127.5, 22.0

INK = '0.1'
CARRIER = '0.3'
GREY_REF = '0.4'
GREY_EVENT = '0.5'
GREY_TEXT = '0.3'
FILL = '0.92'


# -- data --------------------------------------------------------------------------------

def load(run_dir):
    run = metrics.load_run(run_dir)
    d, ev = run['data'], run['events']
    log = open(os.path.join(run_dir, 'logs', 'launch.log')).read()
    hold = float(re.search(r'trajectory held ([0-9.]+) s for the reconfiguration', log).group(1))
    m = re.search(r'weld offset captured for slot (\d+): rho=\(([-0-9.]+),([-0-9.]+)\)', log)
    slot, rho = int(m.group(1)), (float(m.group(2)), float(m.group(3)))
    args = run['manifest']['launch_args']
    pf = os.path.join(run_dir, 'params', 'dissipative_planner.yaml')
    if not os.path.exists(pf):                      # node name before the 3 Oct 2026 rename
        pf = os.path.join(run_dir, 'params', 'dissipative_controller.yaml')
    par = open(pf).read()
    t = d['t']
    sweep = metrics.sweep_window(t, np.column_stack([d['payload_ref_x'], d['payload_ref_y']]))
    t_weld, t_land = ev['WELD'], ev['LAND']
    th = d['payload_tilt_deg']
    # under the level for good: the sample after the last one at or above it before LAND
    above = np.where((t >= t_weld) & (t <= t_land) & (th >= LEVEL_DEG))[0]
    S = dict(d=d, t=t, t0=t[sweep][0], t_done=t[sweep][-1], t_attach=ev['MAGNET'],
             t_weld=t_weld, t_resume=t_weld + hold, t_land=t_land,
             t_handed=t_weld + float(args['attach_t_handout']),
             t_level=t[above[-1] + 1], slot=slot, rho=rho,
             az=[float(a) for a in str(args['attach_azimuths_deg']).split(',')],
             r_ring=float(re.search(r'attach_radius:\s*([0-9.]+)', par).group(1)),
             mass=float(args['load_mass']), rod=float(args['cable_len']),
             n=metrics.n_drones(d))
    # the first sample with the newcomer welded: a frame "at WELD" shows the weld
    S['k_weld'] = int(np.argmax((t >= t_weld) & (d[f'd{slot}_attached'] > 0.5)))
    w, x, y, z = (d['payload_q' + a] for a in 'wxyz')
    S['yaw'] = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    S['err'] = np.hypot(d['payload_x'] - d['payload_ref_x'], d['payload_y'] - d['payload_ref_y'])
    return S


def at(t, te):
    return int(np.searchsorted(t, te))


def rot(d, k):
    w, x, y, z = (d['payload_q' + a][k] for a in 'wxyz')
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pos(d, pre, k):
    return np.array([d[f'{pre}_x'][k], d[f'{pre}_y'][k], d[f'{pre}_z'][k]])


def anchor_load(S, i):
    """Rim anchor of drone i in the load frame (0 deg = 3 o'clock, 90 deg = 12)."""
    if i < len(S['az']):
        a = np.radians(S['az'][i])
        return S['r_ring'] * np.array([np.cos(a), np.sin(a), 0.0])
    return np.array([S['rho'][0], S['rho'][1], 0.0])


def clock(S, i):
    """Clock position of drone i's rim anchor (3 o'clock = +x of the load)."""
    a = np.degrees(np.arctan2(*anchor_load(S, i)[1::-1]))
    return int(round((3 - a / 30.0) % 12)) or 12


# -- (a) snapshots -------------------------------------------------------------------------

class Camera:
    """Orthographic camera; `__call__` maps world offsets to (screen x, screen y, depth),
    depth > 0 towards the viewer. Screen x is horizontal in the world, so a horizontal
    bar on the page is true length."""

    def __init__(self, az, el):
        a, e = np.radians(az), np.radians(el)
        self.c = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        self.r = np.array([-np.sin(a), np.cos(a), 0.0])
        self.u = np.cross(self.c, self.r)

    def __call__(self, p):
        p = np.atleast_2d(p)
        return p @ self.r, p @ self.u, p @ self.c


def scene(S, k):
    """World geometry at sample k, relative to the ring centre."""
    d = S['d']
    P, R = pos(d, 'payload', k), rot(d, k)
    a = np.linspace(0, 2 * np.pi, 361)
    level = S['r_ring'] * np.column_stack([np.cos(a), np.sin(a), 0 * a])
    drones = []
    for i in range(S['n']):
        new = i == S['slot']
        drones.append(dict(i=i, pos=pos(d, f'd{i}', k) - P, anchor=R @ anchor_load(S, i),
                           attached=(not new) or d[f'd{i}_attached'][k] > 0.5, new=new,
                           colour=ps.drone_colour(i) if new else CARRIER))
    return dict(k=k, t=S['t'][k], ring=level @ R.T, level=level, drones=drones,
                tilt=float(d['payload_tilt_deg'][k]))


def draw_scene(ax, sc, cam):
    lx, ly, _ = cam(sc['level'])
    ax.plot(lx, ly, color=GREY_REF, lw=0.7, ls=(0, (1, 1.5)), zorder=1)
    x, y, z = cam(sc['ring'])
    ax.plot(x, np.where(z < 0, y, np.nan), color=ps.PAYLOAD_LINE, lw=0.8, zorder=2)
    ax.plot(x, np.where(z >= 0, y, np.nan), color=ps.PAYLOAD_LINE, lw=1.8, zorder=6)
    arm = 0.075                                  # half-span of a drone's arms (m)
    for dr in sc['drones']:
        ax_, ay_, az_ = (v[0] for v in cam(dr['anchor']))
        dr['anchor_xy'] = (ax_, ay_)
        if not dr['attached']:
            # the reserved rim slot, still empty
            ax.plot(ax_, ay_, 'o', ms=3.4, mfc='white', mec=dr['colour'], mew=0.8, zorder=7)
            sc['slot_xy'] = (ax_, ay_)
            continue
        px, py, pz = (v[0] for v in cam(dr['pos']))
        dr['xy'] = (px, py)
        zo = 4 if (az_ + pz) < 0 else 7
        ax.plot([ax_, px], [ay_, py], color=dr['colour'], lw=0.8, zorder=zo)
        ax.plot(ax_, ay_, 'o', ms=2.6, mfc=dr['colour'], mec='white', mew=0.4, zorder=zo + 0.5)
        zo = 5 if pz < 0 else 8
        ax.plot([px - arm, px + arm], [py, py], color=dr['colour'], lw=1.6,
                solid_capstyle='butt', zorder=zo)
        # the newcomer's hub is filled: it stays the newcomer in greyscale too
        ax.plot(px, py, 'o', ms=3.0, mfc=dr['colour'] if dr['new'] else 'white',
                mec=dr['colour'], mew=0.9, zorder=zo + 0.1)
        if dr['new']:
            sc['new_xy'] = (px, py)


def extents(scenes, cam):
    pts = []
    for sc in scenes:
        x, y, _ = cam(sc['ring'])
        pts += list(zip(x, y))
        for dr in sc['drones']:
            if dr['attached']:
                px, py, _ = cam(dr['pos'])
                pts += [(px[0] - 0.075, py[0]), (px[0] + 0.075, py[0])]
    pts = np.array(pts)
    return pts[:, 0].min(), pts[:, 0].max(), pts[:, 1].min(), pts[:, 1].max()


# -- helpers -------------------------------------------------------------------------------

def badge(ax, xy, n, dx=0, dy=0, coords='data', leader=False):
    """The numbered disc that ties a snapshot in (a) to its place in (b) and (c); with
    `leader`, a hairline from the point to the disc."""
    ax.annotate(rf'\textbf{{{n}}}', xy, xycoords=coords, xytext=(dx, dy),
                textcoords='offset points', ha='center', va='center', fontsize=7,
                color='white', zorder=20,
                bbox=dict(boxstyle='circle,pad=0.16', fc=INK, ec='none'),
                arrowprops=dict(arrowstyle='-', lw=0.4, color=INK, shrinkA=0, shrinkB=1.5)
                if leader else None)


def axes_in(fig, x, y, w, h):
    W, H = fig.get_size_inches()
    return fig.add_axes([x / W, y / H, w / W, h / H])


def arrow(ax, p0, p1, colour, zorder=4, lw=0, head=(3.4, 1.9)):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle='-|>,head_length=%g,head_width=%g' % head,
                                 lw=lw, color=colour, zorder=zorder, mutation_scale=1,
                                 shrinkA=0, shrinkB=0))


def bracket(ax, t0, t1, y, label, gap=0.25):
    """A phase: a hairline with end ticks from t0 to t1, named in a gap at its middle."""
    tk = 1.2                                      # tick length (data units, deg)
    ax.plot([t0 + gap, t0 + gap, t1 - gap, t1 - gap], [y - tk, y, y, y - tk],
            color=INK, lw=0.5, zorder=4, solid_joinstyle='miter')
    ax.text(0.5 * (t0 + t1), y, label, ha='center', va='center', fontsize=8, zorder=5,
            bbox=dict(boxstyle='square,pad=0.15', fc='white', ec='none'))


def tick_g(v, _):
    return f'${v:g}$'


# -- figure --------------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument('run', help="run id or directory, e.g. R0209")
    ap.add_argument('--out', default=OUT, help='output directory')
    a = ap.parse_args()
    run_dir = resolve(a.run)
    rid = os.path.basename(run_dir)[:5]
    S = load(run_dir)
    d, t = S['d'], S['t']
    nw = f"d{S['slot']}_"
    red = ps.drone_colour(S['slot'])
    win = (t >= S['t0']) & (t <= S['t_done'])
    ks = [at(t, 0.5 * (S['t0'] + S['t_attach'])), S['k_weld'], at(t, S['t_resume']),
          at(t, 0.5 * (S['t_resume'] + S['t_done']))]
    k_att = at(t, S['t_attach'])

    ps.thesis()
    W, H = ps.size('full', height=4.1)
    fig = plt.figure(figsize=(W, H))
    fig.set_constrained_layout(False)       # every axes is placed in inches below

    # (a) snapshots --------------------------------------------------------------------
    cam = Camera(VIEW_AZ, VIEW_EL)
    scenes = [scene(S, k) for k in ks]
    x0, x1, y0, y1 = extents(scenes, cam)
    top_h, head, foot = 1.34, 0.22, 0.04     # frame height; header and footer strips (in)
    fw = W / 4
    scale = min(0.94 * fw / (x1 - x0), (top_h - head - foot) / (y1 - y0))   # in per m
    ybase = H - top_h
    frames = []
    for j, sc in enumerate(scenes):
        ax = axes_in(fig, j * fw, ybase, fw, top_h)
        ax.set_axis_off()
        cx = 0.5 * (x0 + x1)
        ax.set_xlim(cx - 0.5 * fw / scale, cx + 0.5 * fw / scale)
        ylo = y0 - foot / scale
        ax.set_ylim(ylo, ylo + top_h / scale)
        draw_scene(ax, sc, cam)
        hx = 0.44 / fw                        # clear of the (a) label in frame 1
        badge(ax, (hx, 1 - 0.5 * head / top_h), j + 1, coords='axes fraction')
        ax.annotate(rf'$t = {sc["t"]:.0f}$\,s', (hx, 1 - 0.5 * head / top_h),
                    xycoords='axes fraction', xytext=(7, 0), textcoords='offset points',
                    ha='left', va='center', fontsize=8)
        # tilt, beside the low point of the ring; bold so the gold holds in greyscale
        rx, ry, _ = cam(sc['ring'])
        kl = int(np.argmin(ry))
        ax.annotate(rf'\boldmath${sc["tilt"]:.0f}^\circ$', (rx[kl], ry[kl]), xytext=(-4, -3),
                    textcoords='offset points', ha='right', va='top', fontsize=8,
                    color=ps.PAYLOAD_LINE)
        frames.append(ax)

    f0, f1 = frames[0], frames[1]
    # names, once: the dotted level ring and the rim clock positions in frame 1, the
    # newcomer in 2
    lx, ly, _ = cam(scenes[0]['level'])
    kx = int(np.argmax(lx))
    f0.annotate('level', (lx[kx], ly[kx]), xytext=(2, -2), textcoords='offset points',
                ha='left', va='top', fontsize=8, color=GREY_TEXT)
    # beside each carrier's anchor, clear of its rod, the ring and the level ring
    beside = {12: (6, 5), 3: (8, 0), 9: (-5, -4)}
    for dr in scenes[0]['drones']:
        if dr['attached']:
            c = clock(S, dr['i'])
            f0.annotate(str(c), dr['anchor_xy'], xytext=beside.get(c, (6, 0)),
                        textcoords='offset points', ha='center', va='center', fontsize=7,
                        color=GREY_TEXT)
    if 'slot_xy' in scenes[0]:
        f0.annotate("6 o'clock", scenes[0]['slot_xy'], xytext=(5, -2),
                    textcoords='offset points', ha='left', va='top', fontsize=8, color=red)
    if 'new_xy' in scenes[1]:
        f1.annotate('newcomer', scenes[1]['new_xy'], xytext=(0, 5), textcoords='offset points',
                    ha='center', va='bottom', fontsize=8, color=red)
    # world x-y-z triad in frame 1's empty lower left, so (a) maps onto (b)
    L = 0.10
    o = np.array([f0.get_xlim()[0] + 0.19 / scale, f0.get_ylim()[0] + 0.05 / scale])
    for vec, name in ((np.eye(3)[0], 'x'), (np.eye(3)[1], 'y'), (np.eye(3)[2], 'z')):
        sx, sy, _ = cam(L * vec)
        tip = o + np.array([sx[0], sy[0]])
        arrow(f0, tuple(o), tuple(tip), INK, zorder=9, lw=0.6, head=(2.6, 1.5))
        f0.annotate(rf'${name}$', tuple(tip), xytext=tuple(4 * np.array([sx[0], sy[0]])
                                                          / np.hypot(sx[0], sy[0])),
                    textcoords='offset points', ha='center', va='center', fontsize=7)
    # scale bar in frame 4 (bottom right, clear of the ring)
    bar = S['r_ring']
    f3 = frames[3]
    xr = f3.get_xlim()[1] - 0.06 / scale
    yb = f3.get_ylim()[0] + 0.10 / scale
    f3.plot([xr - bar, xr], [yb, yb], color=INK, lw=0.9, solid_capstyle='butt')
    f3.text(xr - bar / 2, yb + 0.02 / scale, rf'${bar:g}$\,m', ha='center', va='bottom',
            fontsize=8, color=INK)

    # (b) plan view --------------------------------------------------------------------
    bot = 0.40
    bh = ybase - bot - 0.36
    px, py = d['payload_x'][win], d['payload_y'][win]
    rx, ry = d['payload_ref_x'][win].copy(), d['payload_ref_y'][win].copy()
    # at the weld the held target is moved to the ring: a jump, not a path
    jump = np.r_[False, np.hypot(np.diff(rx), np.diff(ry)) > 0.05]
    rx[jump], ry[jump] = np.nan, np.nan
    kw = S['k_weld']
    Pw, Rw = pos(d, 'payload', kw), rot(d, kw)
    ang = np.linspace(0, 2 * np.pi, 361)
    ring_w = (S['r_ring'] * np.column_stack([np.cos(ang), np.sin(ang), 0 * ang])) @ Rw.T + Pw
    app = (t >= S['t_attach']) & (t <= t[kw])
    nx, ny = d[nw + 'x'][app], d[nw + 'y'][app]

    xs = np.concatenate([px, rx[np.isfinite(rx)], ring_w[:, 0]])
    ys = np.concatenate([py, ry[np.isfinite(ry)], ring_w[:, 1]])
    span = max(np.ptp(xs), np.ptp(ys))
    step = 0.5                                   # one tick step on both axes
    # data plus 5 %, widened to the next tick on the left so x gets three ticks too
    xl = (min(xs.min() - 0.05 * span, step * np.floor(xs.min() / step)),
          xs.max() + 0.05 * span)
    yl = (ys.min() - 0.20 * span, ys.max() + 0.05 * span)
    bw = bh * (xl[1] - xl[0]) / (yl[1] - yl[0])
    bx = 0.50
    axb = axes_in(fig, bx, bot, bw, bh)
    axb.plot(rx, ry, color=GREY_REF, lw=0.8, ls=(0, (3.5, 2.2)), zorder=2)
    axb.plot(px, py, color=ps.PAYLOAD_LINE, lw=1.0, zorder=3)
    axb.plot(ring_w[:, 0], ring_w[:, 1], color=ps.PAYLOAD_LINE, lw=0.6, zorder=3)
    for i, dr in enumerate(scenes[1]['drones']):
        p = Pw + dr['anchor']
        axb.plot(p[0], p[1], 'o', ms=3.0, mfc=dr['colour'], mec='white', mew=0.4, zorder=5)
    axb.plot(nx, ny, color=red, lw=1.0, zorder=4)
    ke = int(np.argmax(ny > yl[0] + 0.06 * (yl[1] - yl[0])))
    arrow(axb, (nx[ke], ny[ke]), (nx[ke + 4], ny[ke + 4]), red)
    axb.annotate('newcomer', (nx[ke], ny[ke]), xytext=(-5, -2.5), textcoords='offset points',
                 ha='right', va='center', fontsize=8, color=red)
    # direction of travel, on the target a sixth of the way round after RESUME
    kr = np.where((t[win] > S['t_resume']) & np.isfinite(rx))[0]
    k1 = kr[len(kr) // 3]
    arrow(axb, (rx[k1 - 3], ry[k1 - 3]), (rx[k1], ry[k1]), GREY_REF, zorder=2)
    # the target held from ATTACH to WELD
    hx_, hy_ = d['payload_ref_x'][k_att], d['payload_ref_y'][k_att]
    axb.plot(hx_, hy_, 'x', ms=4.0, mew=0.9, color=INK, zorder=6)
    # start (open circle) and end (filled square) of the ring's path
    axb.plot(px[0], py[0], 'o', ms=3.2, mfc='white', mec=INK, mew=0.7, zorder=6)
    axb.plot(px[-1], py[-1], 's', ms=2.8, mfc=INK, mec=INK, mew=0, zorder=6)
    axb.annotate('start', (px[0], py[0]), xytext=(-4, 3), textcoords='offset points',
                 ha='right', va='bottom', fontsize=8, color=INK)
    axb.annotate('end', (px[-1], py[-1]), xytext=(-6, -2), textcoords='offset points',
                 ha='right', va='top', fontsize=8, color=INK)
    for j, k in enumerate(ks):
        axb.plot(d['payload_x'][k], d['payload_y'][k], 'o', ms=2.4, color=INK, zorder=7)
    for j, off in enumerate([(11, -9), (-13, 9), (13, -9), (-9, 2)]):
        k = ks[j]
        badge(axb, (d['payload_x'][k], d['payload_y'][k]), j + 1, dx=off[0], dy=off[1],
              leader=True)
    # on the left of the loop at its mid-height, inside it (the target runs outside there)
    mid = 0.5 * (np.nanmin(py) + np.nanmax(py))
    left = np.where(px < np.nanmean(px), np.abs(py - mid), np.inf)
    kl = int(np.argmin(left))
    axb.annotate(r'\textbf{ring}', (px[kl], py[kl]), xytext=(5, 0), textcoords='offset points',
                 ha='left', va='center', fontsize=8, color=ps.PAYLOAD_LINE)
    # the target, where it is furthest from the centre of the loop towards the upper right
    cxy = np.array([np.nanmean(px), np.nanmean(py)])
    kt = int(np.nanargmax((rx - cxy[0]) + (ry - cxy[1])))
    axb.annotate('target', (rx[kt], ry[kt]), xytext=(3, 3), textcoords='offset points',
                 ha='left', va='bottom', fontsize=8, color=GREY_TEXT)
    # the ring outline at the weld, named inside (b) in the empty corner below it
    kb = int(np.argmin(np.abs(ang - np.radians(-20))))
    axb.annotate('ring at\n' + r'\textsc{weld}', (ring_w[kb, 0], ring_w[kb, 1]),
                 xytext=(xl[1] - 0.01 * span, ring_w[:, 1].min() - 0.08 * span),
                 textcoords='data', ha='right', va='top', fontsize=8, color=GREY_TEXT,
                 linespacing=1.0,
                 arrowprops=dict(arrowstyle='-', lw=0.4, color=GREY_TEXT, shrinkA=0,
                                 shrinkB=0))
    axb.set_xlim(*xl)
    axb.set_ylim(*yl)
    axb.set_aspect('equal', adjustable='box')
    for axis in (axb.xaxis, axb.yaxis):
        axis.set_major_locator(plt.MultipleLocator(step))
        axis.set_major_formatter(plt.FuncFormatter(tick_g))
    axb.set_xlabel(r'$x$ (m)')
    axb.set_ylabel(r'$y$ (m)')

    # (c) ring tilt --------------------------------------------------------------------
    cx0 = bx + bw + 0.62
    axc = axes_in(fig, cx0, bot, W - cx0 - 0.06, bh)
    tt, th = t[win], d['payload_tilt_deg'][win]
    TOP = 50.0
    axc.axvspan(S['t_attach'], S['t_resume'], color=FILL, lw=0, zorder=0)
    for name, te in (('attach', S['t_attach']), ('weld', S['t_weld']),
                     ('resume', S['t_resume'])):
        axc.axvline(te, color=GREY_EVENT, lw=0.5, zorder=1)
        axc.annotate(rf'\textsc{{{name}}}', (te, 1.0), xycoords=('data', 'axes fraction'),
                     xytext=(0, 3), textcoords='offset points', ha='center', va='bottom',
                     fontsize=9)
    # phases along the top: who carries the ring
    yph = TOP - 3.5
    bracket(axc, S['t0'], S['t_weld'], yph, '3 drones')
    bracket(axc, S['t_weld'], S['t_handed'], yph, 'hand-out')
    bracket(axc, S['t_handed'], S['t_done'], yph, '4 drones')
    axc.axhline(LEVEL_DEG, color=GREY_REF, lw=0.8, ls=(0, (3.5, 2)), zorder=2)
    axc.annotate(rf'${LEVEL_DEG:g}^\circ$', (S['t0'], LEVEL_DEG), xytext=(3, 2),
                 textcoords='offset points', ha='left', va='bottom', fontsize=8,
                 color=GREY_TEXT)
    axc.plot(tt, th, color=ps.PAYLOAD_LINE, lw=1.0, zorder=3, rasterized=tt.size > 2000)
    # the overshoot after the weld, named at its peak
    mh = (t >= S['t_weld']) & (t <= S['t_handed'])
    kp = int(np.flatnonzero(mh)[np.argmax(d['payload_tilt_deg'][mh])])
    axc.annotate(rf'${d["payload_tilt_deg"][kp]:.0f}^\circ$', (t[kp], d['payload_tilt_deg'][kp]),
                 xytext=(-3, 0), textcoords='offset points', ha='right', va='center',
                 fontsize=8, color=INK)
    # WELD -> the last crossing of the level, which gets an open circle on the line
    yb = LEVEL_DEG + 3.0
    axc.annotate('', (S['t_level'], yb), xytext=(S['t_weld'], yb),
                 arrowprops=dict(arrowstyle='<|-|>', mutation_scale=5, lw=0.5, color=INK,
                                 shrinkA=0, shrinkB=0), zorder=4)
    axc.plot([S['t_level']] * 2, [yb, LEVEL_DEG], color=INK, lw=0.4, zorder=4)
    axc.plot(S['t_level'], LEVEL_DEG, 'o', ms=3.4, mfc='white', mec=INK, mew=0.7, zorder=7)
    axc.annotate(rf'${S["t_level"] - S["t_weld"]:.1f}$\,s', (S['t_weld'], yb),
                 xytext=(3, 2), textcoords='offset points', ha='left', va='bottom',
                 fontsize=8)
    # the level the four drones hold, under the oscillation
    after = (tt >= S['t_level'])
    mean4 = float(np.nanmean(th[after]))
    axc.plot([S['t_level'], S['t_done']], [mean4, mean4], color=INK, lw=0.8, zorder=5,
             solid_capstyle='butt')
    xm = S['t_done'] - 0.18 * (S['t_done'] - S['t_level'])
    axc.annotate(rf'mean ${mean4:.1f}^\circ$', (xm, mean4), xytext=(xm, LEVEL_DEG + 4),
                 textcoords='data', ha='center', va='bottom', fontsize=8,
                 arrowprops=dict(arrowstyle='-', lw=0.4, color=INK, shrinkA=1, shrinkB=0))
    axc.text(0.5 * (S['t_attach'] + S['t_weld']), 20, 'target held', ha='center',
             va='center', fontsize=8, color=GREY_TEXT)
    for j, k in enumerate(ks):
        axc.plot(t[k], d['payload_tilt_deg'][k], 'o', ms=2.6, color=INK, zorder=6)
    # one convention: each badge on a short slanted leader, on the side away from the
    # event line its snapshot sits on (2 on WELD, 3 on RESUME)
    for j, k in enumerate(ks):
        xy = (t[k], d['payload_tilt_deg'][k])
        tgt = (t[k] - 1.5, xy[1] + 6.5) if j < 2 else (t[k] + 1.5, LEVEL_DEG + 7.5)
        axc.annotate(rf'\textbf{{{j + 1}}}', xy, xytext=tgt, textcoords='data',
                     ha='center', va='center', fontsize=7, color='white', zorder=20,
                     bbox=dict(boxstyle='circle,pad=0.16', fc=INK, ec='none'),
                     arrowprops=dict(arrowstyle='-', lw=0.4, color=INK, shrinkA=0,
                                     shrinkB=1.5))
    axc.set_xlim(S['t0'], S['t_done'])
    axc.set_ylim(0, TOP)
    axc.set_yticks(np.arange(0, TOP - 1, 10))
    axc.xaxis.set_major_locator(plt.MultipleLocator(10))
    axc.xaxis.set_major_formatter(plt.FuncFormatter(tick_g))
    axc.yaxis.set_major_formatter(plt.FuncFormatter(tick_g))
    axc.set_xlabel(r'$t$ (s)')
    axc.set_ylabel(r'Ring tilt $\theta$ (\si{\degree})')

    # panel labels on each row's first line of text: the frame headers, the event names
    ytop = (bot + bh + 3 / 72) / H
    fig.text(0.004, 1 - 0.5 * head / H, '(a)', ha='left', va='center', fontsize=9)
    fig.text(0.004, ytop, '(b)', ha='left', va='bottom', fontsize=9)
    fig.text((cx0 - 0.50) / W, ytop, '(c)', ha='left', va='bottom', fontsize=9)

    stem = os.path.join(a.out, f'attach_storyboard_{rid}')
    print('\n'.join(ps.save_print(fig, stem)))
    report(S, ks)


def report(S, ks):
    """Every number the caption quotes, from the run."""
    d, t, err, yaw = S['d'], S['t'], S['err'], S['yaw']
    th = d['payload_tilt_deg']
    rng = lambda lo, hi: (t >= lo) & (t <= hi)                      # noqa: E731
    pre = rng(S['t0'], t[S['k_weld']])
    last2 = rng(S['t_weld'] - 2, S['t_weld'])
    after = rng(S['t_level'], S['t_done'])
    ho = rng(S['t_weld'], S['t_handed'])
    kp = int(np.flatnonzero(ho)[np.argmax(th[ho])])
    k_res = at(t, S['t_resume'])
    kw = S['k_weld']
    # oscillation after the level: the strongest spectral line above 0.5 Hz
    x = th[after] - th[after].mean()
    f = np.fft.rfftfreq(x.size, np.median(np.diff(t[after])))
    P = np.abs(np.fft.rfft(x)) ** 2
    f_osc = f[f > 0.5][np.argmax(P[f > 0.5])]
    rms = lambda m: float(np.sqrt(np.mean(err[m] ** 2)))            # noqa: E731
    print(f"mass {S['mass']:g} kg, rod {S['rod']:g} m, ring radius {S['r_ring']:g} m; "
          f"carriers at {S['az']} deg, newcomer slot {S['slot']} rho={S['rho']}")
    print(f"sweep {S['t0']:.2f}-{S['t_done']:.2f} s; ATTACH {S['t_attach']:.2f}, "
          f"WELD {S['t_weld']:.2f}, hand-out done {S['t_handed']:.2f}, "
          f"RESUME {S['t_resume']:.2f}, LAND {S['t_land']:.2f}")
    print(f"tilt before WELD: mean {np.nanmean(th[pre]):.1f} deg "
          f"(range {np.nanmin(th[pre]):.1f}-{np.nanmax(th[pre]):.1f})")
    print(f"ring from the held target at WELD: {err[kw - 1]:.3f} m; ring moved "
          f"{1e3 * np.hypot(np.ptp(d['payload_x'][last2]), np.ptp(d['payload_y'][last2])):.0f}"
          f" mm in the 2 s before WELD")
    print(f"peak after WELD {th[kp]:.1f} deg at {t[kp]:.2f} s = WELD + {t[kp] - S['t_weld']:.1f} s")
    print(f"under {LEVEL_DEG:g} deg from {S['t_level']:.2f} s = WELD + "
          f"{S['t_level'] - S['t_weld']:.2f} s (RESUME - {S['t_resume'] - S['t_level']:.2f} s);"
          f" after: max {np.nanmax(th[after]):.2f} deg at {t[after][np.argmax(th[after])]:.2f} s,"
          f" mean {np.nanmean(th[after]):.2f}, oscillation {f_osc:.2f} Hz")
    print(f"ring drift WELD -> RESUME {np.hypot(d['payload_x'][k_res] - d['payload_x'][kw], d['payload_y'][k_res] - d['payload_y'][kw]):.3f} m"
          f" (dy {d['payload_y'][k_res] - d['payload_y'][kw]:+.3f})")
    print(f"ring to target: 3 drones {S['t0']:.1f}-ATTACH rms {rms(rng(S['t0'], S['t_attach'])):.3f} m;"
          f" RESUME-end rms {rms(rng(S['t_resume'], S['t_done'])):.3f} m, "
          f"max {err[rng(S['t_resume'], S['t_done'])].max():.3f} m")
    print(f"yaw: max |yaw| before WELD {np.abs(yaw[pre]).max():.1f}, hand-out "
          f"{np.abs(yaw[ho]).max():.1f}, after RESUME {np.abs(yaw[rng(S['t_resume'], S['t_done'])]).max():.1f};"
          f" at the snapshots " + ', '.join(f'{yaw[k]:.1f}' for k in ks))
    print('snapshots ' + ', '.join(f"{t[k]:.2f} s {th[k]:.1f} deg" for k in ks))


if __name__ == '__main__':
    main()
