#!/usr/bin/env python3
"""tools/twin_video.py -- a twin (Gazebo) run rendered from its logs as a presentation video.

    python3 tools/twin_video.py RUNID [--out PATH] [--speed 1.0] [--t0 T] [--t1 T]
                                [--title "..."] [--fps 30] [--decimate N] [--view AZ,EL]
                                [--still T [T ...]]

1920x1080 H.264 (yuv420p, +faststart: plays in PowerPoint, Keynote and a browser), 30 fps, real
time unless --speed (2 = twice as fast, 0.5 = slow motion).

  left    the fleet, the ring and the rods in a fixed orthographic 3-D view rebuilt from run.csv: one
          camera and one scale for the whole video (fitted to what flies in the window), a faint floor
          grid with the drones' and the ring's shadows when something comes within 0.35 m of the floor;
          the ring's far half drawn lighter
  right   ring tilt (deg) and ring height (m) over the shown window, a moving cursor, the run's events
          as thin lines with short labels (detach, contact, attach confirmed = WELD, hand-over = MUX:ours,
          LAND, landed, take-back, give-up, ball pickup, ball drop)
  top     sim time, the speed, the fleet's state and, in partner runs, Tejen's phase (PHASE: events);
          each event also flashes for 2.5 s at the top of the 3-D view
  bottom  the caption (--title) and the run id

Window: --t0 / --t1 take sim seconds or an event plus an offset: TAKEOFF-1, LANDED+2, CONTACT-5 (the
first CONTACT), CONTACT#2+1 (the second), WELD+10, start, end. Default: the whole log.
--decimate N renders every Nth frame (a quick preview at fps/N). --still T ... writes PNG frames at those
sim times instead of a video (the same drawing code). Rendering is ~10-20 ms a frame on an idle laptop;
while Gazebo runs, prefix `nice -n 19` (ffmpeg inherits it and is held to 2 threads).

Geometry, all from the run directory (manifest launch args, else params/dissipative_planner.yaml):
carriers d0..d{num_drones-1} sit on the rim at attach_azimuths_deg, radius attach_radius, height attach_z in
the ring frame, so a carrier's plate is ring pose + R(q) @ (r cos az, r sin az, attach_z); its rod runs from
its pivot (body + pivot_offset_z) to that plate, until a RELEASE or DETACH names it (it flies off with its
rod, whose angle is not logged, so the rod is no longer drawn). The column block after the carriers
(d{num_drones}) is QUAD4: its rod runs to its magnet tip (tip_x/y/z), and its plate (attach_x_offset,
attach_y_offset, when the launch args give them) is an open square on the ring. In a detach run the released
carrier takes the highlight colour. Colours: plot_style (RViz) drone colour for the highlighted drone,
carriers dark grey, ring plot_style.PAYLOAD_LINE. Drone bodies are drawn level (no attitude is logged
beyond the tilt magnitude); a disarmed drone is drawn faded with a hollow hub.

README: the videos for the supervisor meeting (Tue 13 Oct 2026), made on 10 Oct from the main tree
------
    source ~/ros2_humble/install/setup.bash && source install/setup.bash
    T=tools/twin_video.py; V=results/videos
    nice -n 19 python3 $T R1459 --t0 TAKEOFF-1 --t1 LANDED+2 --out $V/w4_hover_detach_R1459.mp4 \\
        --title "W4 hover detach: drone 1 lets go of the ring, lands and is disarmed alone; the other three carry on"
    nice -n 19 python3 $T R1457 --t0 TAKEOFF-1 --t1 LANDED+2 --out $V/w5_circle_detach_R1457.mp4 \\
        --title "W5 circle detach: drone 1 lets go while the ring flies a circle of radius 0.5 m; the other three carry on"
    nice -n 19 python3 $T R1458 --t0 TAKEOFF-1 --t1 LANDED+2 --out $V/r8a0_carry_three_R1458.mp4 \\
        --title "R8a0: three drones carry the ring while QUAD4 stays armed and idle on the floor"
    nice -n 19 python3 $T R1487 --speed 2 --out $V/m1_subtest_hover_R1487_x2.mp4 \\
        --title "Combined M1, hover: QUAD4 fetches the ball, drops it in the ring and attaches; we take it over and land"
    nice -n 19 python3 $T R1487 --t0 CONTACT-5 --t1 WELD+10 --speed 0.5 \\
        --out $V/m1_subtest_hover_attach_closeup_R1487_x0.5.mp4 \\
        --title "Close-up: the first lift proof times out, the second confirms the attach; hand-over and fold-in"
    nice -n 19 python3 $T R1488 --speed 2 --out $V/m1_subtest_circle_R1488_x2.mp4 \\
        --title "Combined M1 on the circle: the same mission while the ring circles (radius 0.5 m, 0.125 m/s)"
    nice -n 19 python3 $T R1481 --t0 CONTACT-5 --t1 TAKE_BACK+20 --out $V/m1_subtest_nocatch_takeback_R1481.mp4 \\
        --title "No catch: three lift proofs fail, Tejen's planner gives up, we take QUAD4 back and land it"
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import time

import numpy as np
import pandas as pd

sys.dont_write_bytecode = True            # reading the repo must not write into it
TOOLS = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, TOOLS)
import plot_style as ps                                            # noqa: E402  (selects Agg)
import metrics                                                     # noqa: E402
from run_dir import resolve                                        # noqa: E402
import matplotlib.pyplot as plt                                    # noqa: E402
from matplotlib.lines import Line2D                                # noqa: E402
from matplotlib.patches import Polygon, Rectangle                  # noqa: E402

W_PX, H_PX, DPI = 1920, 1080, 100
VIEW = (-127.5, 24.0)             # viewer azimuth about z from +x, elevation (deg): from the south-west
ARM = 0.11                        # drone arm half-span drawn (m)
PROP = 0.05                       # prop ring radius drawn (m)
FLASH_S = 2.5                     # an event's banner stays this long (video seconds)
MERGE_S = 0.2                     # events closer than this share one label
GRID_M = 0.5
FLOOR_Z = 0.35                    # the floor is drawn when a drone or the ring comes this low in the window

INK = '#1E2633'
GREY = '#6A7179'
CARRIER = '#4A4F57'
FLOOR = '#E3E6EA'
SHADOW = '#C3C8CE'
RING = ps.PAYLOAD_LINE
BALL = '#2C5AA0'

# runner event -> (short label, colour); the order is the order inside a merged label
EVENTS = {
    'PICKUP': ('ball pickup', '#8B5A2B'),
    'DROP': ('ball drop', '#8B5A2B'),
    'CONTACT': ('contact', '#2F6DB5'),
    'WELD': ('attach confirmed', '#2E8B3E'),
    'MUX:ours': ('hand-over', '#2E8B3E'),
    'GIVE_UP': ('give-up', '#C0392B'),
    'TAKE_BACK': ('take-back', '#8E44AD'),
    'RELEASE': ('detach', '#D35400'),
    'DETACH': ('detach', '#D35400'),
    'LAND': ('LAND', GREY),
    'LANDED': ('landed', GREY),
}
ONCE = {'GIVE_UP', 'TAKE_BACK', 'MUX:ours', 'LAND', 'LANDED'}     # the runner can log these at 5 Hz
FLEET = (('ARM', 'armed on the floor'), ('TAKEOFF', 'taking off'), ('LIFTED', 'carrying the ring'),
         ('LAND', 'landing'), ('LANDED', 'landed'), ('FLEET_ABORT', 'fleet abort'))


# -- the run ---------------------------------------------------------------------------------------

def _param(text, name):
    m = re.search(rf'^\s*{name}:\s*(.+?)\s*$', text or '', re.M)
    return m.group(1).strip('\'"') if m else None


def load(spec):
    """The run as one dict: the table, the events and the geometry."""
    d = resolve(spec)
    run = pd.read_csv(os.path.join(d, 'logs', 'run.csv'))
    run = run.drop_duplicates('t').sort_values('t').reset_index(drop=True)
    man = json.load(open(os.path.join(d, 'manifest.json')))
    la = man.get('launch_args') or {}
    par = ''
    for name in ('dissipative_planner.yaml', 'dissipative_controller.yaml'):
        p = os.path.join(d, 'params', name)
        if os.path.exists(p):
            par = open(p).read()
            break

    def arg(name, default=None):
        v = la.get(name)
        if v is None or v == '':
            v = _param(par, name)
        return default if v is None else v

    n_car = int(arg('num_drones', 3))
    n_all = metrics.n_drones(run)
    az = arg('attach_azimuths_deg')
    if az is None:
        raise SystemExit(f'{spec}: no attach_azimuths_deg in the launch args or the planner params')
    az = [float(a) for a in str(az).strip('[]').split(',')][:n_car]
    events = [e for e in metrics.read_event_list(d) if math.isfinite(e[0])]
    released = {}
    for t, name, a in events:
        if name in ('RELEASE', 'DETACH'):
            m = re.search(r'/drone_(\d+)', a)
            try:
                i = int(m.group(1)) if m else int(float(a))
            except ValueError:
                continue
            released.setdefault(i, t)
    if n_all > n_car:
        hi, hi_name = n_car, 'QUAD4'
    elif released:
        hi = min(released, key=released.get)
        hi_name = f'drone {hi} (released)'
    else:
        hi, hi_name = None, None
    plate_new = None
    if la.get('attach_x_offset') is not None and la.get('attach_y_offset') is not None:
        plate_new = (float(la['attach_x_offset']), float(la['attach_y_offset']))
    cfg = next((os.path.splitext(os.path.basename(a))[0] for a in man.get('argv', [])
                if a.endswith('.yaml')), '')
    return dict(dir=d, id=os.path.basename(d).split('_')[0], cfg=cfg, run=run, t=run.t.to_numpy(float),
                events=events, n_car=n_car, n_all=n_all, az=az,
                r=float(arg('attach_radius', 0.225)), z_att=float(arg('attach_z', 0.02)),
                pivot=float(arg('pivot_offset_z', -0.04)), released=released, hi=hi, hi_name=hi_name,
                plate_new=plate_new, has_tip='tip_x' in run, has_ball='ball_x' in run,
                partner=any(e[1].startswith('PHASE:') for e in events))


def when(spec, events, t_first, t_last):
    """Sim time of '12.5', 'start', 'end', 'CONTACT', 'CONTACT-5', 'CONTACT#2+1', 'MUX:ours+2'."""
    s = str(spec).strip()
    try:
        return float(s)
    except ValueError:
        pass
    m = re.fullmatch(r'([A-Za-z_:]+?)(?:#(\d+))?([+-]\d+(?:\.\d*)?)?', s)
    if not m:
        raise SystemExit(f'cannot read the time {spec!r}')
    name, k, off = m.group(1), int(m.group(2) or 1), float(m.group(3) or 0.0)
    if name.lower() == 'start':
        base = t_first
    elif name.lower() == 'end':
        base = t_last
    else:
        times = [e[0] for e in events if e[1] == name]
        if len(times) < k:
            raise SystemExit(f'event {name} #{k} not in this run (it has {len(times)})')
        base = times[k - 1]
    return base + off


def event_groups(events, t0=-math.inf, t1=math.inf):
    """The labelled events in [t0, t1]: [{t, names, label, colour}], events within MERGE_S merged
    ('attach confirmed, hand-over'); the switch that IS a take-back is not also called a hand-over."""
    seen, pts = set(), []
    for t, name, _ in events:
        if name not in EVENTS or (name in ONCE and name in seen):
            continue
        seen.add(name)
        pts.append((t, name))
    pts.sort()
    groups = []
    for t, name in pts:
        if groups and t - groups[-1]['last'] <= MERGE_S:
            groups[-1]['names'].append(name)
            groups[-1]['last'] = t
        else:
            groups.append({'t': t, 'last': t, 'names': [name]})
    order = list(EVENTS)
    out = []
    for g in groups:
        names = sorted(set(g['names']), key=order.index)
        if 'TAKE_BACK' in names and 'MUX:ours' in names:
            names.remove('MUX:ours')
        labels = []
        for n in names:
            if EVENTS[n][0] not in labels:
                labels.append(EVENTS[n][0])
        out.append({'t': g['t'], 'names': names, 'label': ', '.join(labels), 'colour': EVENTS[names[0]][1]})
    return [g for g in out if t0 <= g['t'] <= t1]


# -- geometry ----------------------------------------------------------------------------------------

def quat_rot(q):
    """(n, 4) w x y z -> (n, 3, 3) body -> world."""
    q = np.asarray(q, float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0], R[:, 0, 1], R[:, 0, 2] = 1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)
    R[:, 1, 0], R[:, 1, 1], R[:, 1, 2] = 2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)
    R[:, 2, 0], R[:, 2, 1], R[:, 2, 2] = 2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)
    return R


def rim(S, az_deg):
    """A point on the ring at azimuth az (load frame, 0 = +x), at the plates' height."""
    a = np.radians(az_deg)
    return np.array([S['r'] * np.cos(a), S['r'] * np.sin(a), S['z_att']])


def ring_circle(S, k=145):
    a = np.linspace(0.0, 2 * np.pi, k)
    return np.column_stack([S['r'] * np.cos(a), S['r'] * np.sin(a), np.full(k, S['z_att'])])


class Camera:
    """Orthographic: screen x is horizontal in the world (a horizontal bar is true length), depth > 0
    towards the viewer."""

    def __init__(self, az, el):
        a, e = np.radians(az), np.radians(el)
        self.c = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
        self.r = np.array([-np.sin(a), np.cos(a), 0.0])
        self.u = np.cross(self.c, self.r)

    def xy(self, p):
        p = np.asarray(p, float)
        return p @ self.r, p @ self.u

    def depth(self, p):
        return np.asarray(p, float) @ self.c


class Track:
    """Everything the frames need, sampled at the frame times: positions linearly, the quaternion
    linearly then normalised (sign kept continuous), arming as the last logged value."""

    def __init__(self, S, tf):
        run, t = S['run'], S['t']
        self.tf = tf

        def lin(col):
            if col not in run:
                return np.full(len(tf), np.nan)
            return np.interp(tf, t, run[col].to_numpy(float))

        def xyz(pre):
            return np.column_stack([lin(f'{pre}_{a}') for a in 'xyz'])

        idx = np.clip(np.searchsorted(t, tf, 'right') - 1, 0, len(t) - 1)
        self.P = xyz('payload')
        q = run[['payload_qw', 'payload_qx', 'payload_qy', 'payload_qz']].to_numpy(float)
        flip = np.r_[1.0, np.cumprod(np.where(np.sum(q[1:] * q[:-1], 1) < 0, -1.0, 1.0))]
        q = q * flip[:, None]
        self.R = quat_rot(np.column_stack([np.interp(tf, t, q[:, j]) for j in range(4)]))
        self.D = [xyz(f'd{i}') for i in range(S['n_all'])]
        self.armed = [run[f'd{i}_armed'].to_numpy(float)[idx] > 0.5 if f'd{i}_armed' in run
                      else np.ones(len(tf), bool) for i in range(S['n_all'])]
        self.tip = xyz('tip') if S['has_tip'] else None
        self.ball = xyz('ball') if S['has_ball'] else None


def plates(S, P, R):
    """World plate points of the carriers for one frame: ring pose + R @ rim(az)."""
    return [P + R @ rim(S, a) for a in S['az']]


# -- the figure --------------------------------------------------------------------------------------

def fx(px):
    return px / W_PX


def fy(py):
    return py / H_PX


def speed_words(speed):
    if abs(speed - 1.0) < 1e-9:
        return 'real time'
    return f'{speed:g}x real time' + (' (slow motion)' if speed < 1 else '')


def phase_words(name):
    return name.split(':', 1)[1].replace('_', ' ').lower()


class Video:
    def __init__(self, S, t0, t1, tf, speed, title, view):
        self.S, self.t0, self.t1, self.speed = S, t0, t1, speed
        self.tr = Track(S, tf)
        self.cam = Camera(*view)
        self.circ = ring_circle(S)
        self.groups = event_groups(S['events'], t0, t1)
        self.phases = [(t, phase_words(n)) for t, n, _ in S['events'] if n.startswith('PHASE:')]
        self.fleet = [(t, w) for t, n, _ in S['events'] for name, w in FLEET if n == name]
        self.fleet.sort()
        plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 14, 'axes.linewidth': 0.8})
        self.fig = plt.figure(figsize=(W_PX / DPI, H_PX / DPI), dpi=DPI, facecolor='white')
        self.anim = []
        self._scene()
        self._plots()
        self._text(title)
        for a in self.anim:
            a.set_animated(True)
        self.fig.canvas.draw()
        self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)

    def _add(self, artist, z):
        artist.set_zorder(z)
        self.anim.append(artist)
        return artist

    # -- left: the 3-D view
    def _scene(self):
        S, cam = self.S, self.cam
        # the scene's extent over every logged row of the window fixes the camera for the video (and
        # for --still frames, which then match it)
        t = S['t']
        ex = Track(S, t[(t >= self.t0) & (t <= self.t1)])
        ring_pts = (ex.P[:, None, :] + np.einsum('nij,kj->nki', ex.R, self.circ[::12])).reshape(-1, 3)
        pts = [ring_pts] + [D for D in ex.D] + ([ex.tip] if ex.tip is not None else [])
        pts = np.vstack(pts)
        pts = pts[np.all(np.isfinite(pts), 1)]
        lo = np.floor((pts[:, :2].min(0) - 0.3) / GRID_M) * GRID_M
        hi = np.ceil((pts[:, :2].max(0) + 0.3) / GRID_M) * GRID_M
        self.box = (lo, hi)
        self.floor = bool(np.nanmin(pts[:, 2]) < FLOOR_Z)
        gx, gy = np.arange(lo[0], hi[0] + 1e-6, GRID_M), np.arange(lo[1], hi[1] + 1e-6, GRID_M)
        # fitted to what flies in the window: the floor shows only when something comes down to it
        sx, sy = cam.xy(pts)
        sx = np.r_[sx - ARM - PROP - 0.05, sx + ARM + PROP + 0.05]
        sy = np.r_[sy - 0.08, sy + 0.08]
        self.quad = self._quad_template()

        L, B, Wd, Ht = 0, 64, 1310, 1016             # the 3-D axes (px)
        top, bottom, side = 175, 64, 30               # kept free for the text and the legend
        self.ax = ax = self.fig.add_axes([fx(L), fy(B), fx(Wd), fy(Ht)])
        ax.set_axis_off()
        cw, ch = Wd - 2 * side, Ht - top - bottom
        span_x, span_y = sx.max() - sx.min(), sy.max() - sy.min()
        s = max(span_x / cw, span_y / ch)             # metres per pixel
        cx, cy = 0.5 * (sx.max() + sx.min()), 0.5 * (sy.max() + sy.min())
        cy_axes = cy + s * (0.5 * (top - bottom))     # content centred in the free box
        ax.set_xlim(cx - s * Wd / 2, cx + s * Wd / 2)
        ax.set_ylim(cy_axes - s * Ht / 2, cy_axes + s * Ht / 2)
        self.m_per_px = s

        segs = []
        for x in gx:
            segs += [[x, lo[1], 0], [x, hi[1], 0], [np.nan] * 3]
        for y in gy:
            segs += [[lo[0], y, 0], [hi[0], y, 0], [np.nan] * 3]
        gxs, gys = cam.xy(np.array(segs))
        # the floor and the shadows stay out of the legend's band
        clip = Rectangle((cx - s * (Wd / 2 - side), cy_axes - s * (Ht / 2 - bottom)), s * (Wd - 2 * side),
                         s * (Ht - bottom), transform=ax.transData, fill=False, visible=False)
        ax.add_patch(clip)
        grid = ax.plot(gxs, gys, color=FLOOR, lw=1.0, zorder=0)[0]
        grid.set_clip_path(clip)
        grid.set_visible(self.floor)

        # scale bar, bottom right of the view
        x1 = cx + s * (Wd / 2 - side - 10)
        y1 = cy_axes - s * (Ht / 2 - bottom + 22)
        ax.plot([x1 - GRID_M, x1], [y1, y1], color=INK, lw=2.5, solid_capstyle='butt', zorder=1)
        ax.text(x1 - GRID_M / 2, y1 + s * 8, f'{GRID_M:g} m', ha='center', va='bottom', fontsize=14,
                color=INK)

        self.sh_ring = self._add(ax.plot([], [], color=SHADOW, lw=2.0, alpha=0.8)[0], 1)
        self.sh_drones = self._add(ax.plot([], [], 'o', ms=7, color=SHADOW, alpha=0.9, ls='none')[0], 1)
        self.sh_ring.set_clip_path(clip)
        self.sh_drones.set_clip_path(clip)
        self.sh_ring.set_visible(self.floor)
        self.sh_drones.set_visible(self.floor)
        self.ring_fill = self._add(Polygon(np.zeros((3, 2)), closed=True, fc=RING, ec='none', alpha=0.12), 2)
        ax.add_patch(self.ring_fill)
        far = _mix(RING, '#FFFFFF', 0.55)
        self.ring_far = self._add(ax.plot([], [], color=far, lw=3.0, solid_capstyle='round')[0], 3)
        self.ring_near = self._add(ax.plot([], [], color=RING, lw=5.0, solid_capstyle='round')[0], 6)
        self.colours = [ps.drone_colour(i) if i == S['hi'] else CARRIER for i in range(S['n_all'])]
        self.rods, self.arms, self.props, self.hubs = [], [], [], []
        for i in range(S['n_all']):
            c = self.colours[i]
            self.rods.append(self._add(ax.plot([], [], color=c, lw=2.2, marker='o', ms=6, markevery=[1],
                                               mfc=c, mec='white', mew=1.0)[0], 4))
            self.arms.append(self._add(ax.plot([], [], color=c, lw=3.5, solid_capstyle='round')[0], 5))
            self.props.append(self._add(ax.plot([], [], color=c, lw=1.6)[0], 5))
            self.hubs.append(self._add(ax.plot([], [], 'o', ms=9, color=c, mec=c, mew=2.0)[0], 5))
        hc = ps.drone_colour(S['hi']) if S['hi'] is not None else INK
        self.plate_new = self._add(ax.plot([], [], 's', ms=9, mfc='white', mec=hc, mew=2.0)[0], 7)
        self.tip = self._add(ax.plot([], [], 'o', ms=8, mfc=hc, mec='white', mew=1.2)[0], 8)
        self.ball = self._add(ax.plot([], [], 'o', ms=13, mfc=BALL, mec='white', mew=1.2)[0], 9)

        handles = [Line2D([], [], color=CARRIER, lw=4, label='carrier')]
        if S['hi'] is not None:
            handles.append(Line2D([], [], color=hc, lw=4, label=S['hi_name']))
        handles.append(Line2D([], [], color=RING, lw=5, label='ring'))
        if S['has_tip']:
            handles.append(Line2D([], [], color=hc, marker='o', ls='none', ms=8, mec='white', label='magnet tip'))
        if S['plate_new'] is not None:
            handles.append(Line2D([], [], marker='s', ls='none', ms=9, mfc='white', mec=hc, mew=2,
                                  label="QUAD4's plate"))
        if S['has_ball']:
            handles.append(Line2D([], [], color=BALL, marker='o', ls='none', ms=12, mec='white', label='ball'))
        ax.legend(handles=handles, loc='lower left', bbox_to_anchor=(fx(side) / fx(Wd), 0.0), ncol=len(handles),
                  frameon=False, fontsize=15, handlelength=1.6, columnspacing=1.4, borderaxespad=0.3)

    def _quad_template(self):
        """A level quadcopter at the origin in screen coordinates: X arms on the camera's diagonals
        (symmetric on screen) and a prop ring at each arm's end."""
        cam = self.cam
        h = np.array([cam.c[0], cam.c[1], 0.0])
        h /= np.linalg.norm(h)
        d1, d2 = (cam.r + h) / np.sqrt(2), (cam.r - h) / np.sqrt(2)
        nan = np.full(3, np.nan)
        # the arms, then the mast down to the rod's pivot
        arms = np.array([ARM * d1, -ARM * d1, nan, ARM * d2, -ARM * d2, nan, 0 * d1,
                         [0.0, 0.0, self.S['pivot']]])
        a = np.linspace(0, 2 * np.pi, 25)
        ring = PROP * np.column_stack([np.cos(a), np.sin(a), 0 * a])
        props = np.vstack([np.vstack([e + ring, nan]) for e in (ARM * d1, -ARM * d1, ARM * d2, -ARM * d2)])
        return np.column_stack(cam.xy(arms)), np.column_stack(cam.xy(props))

    # -- right: tilt and height
    def _plots(self):
        S, t0, t1 = self.S, self.t0, self.t1
        run, t = S['run'], S['t']
        w = (t >= t0 - 0.5) & (t <= t1 + 0.5)
        self.tw = t[w]
        x0, wid, h = 1405, 480, 300
        axT = self.fig.add_axes([fx(x0), fy(505), fx(wid), fy(h)])
        axZ = self.fig.add_axes([fx(x0), fy(150), fx(wid), fy(h)], sharex=axT)
        self.axT, self.axZ = axT, axZ
        self.series = []
        for ax, col, name, unit, fmt, floor in ((axT, 'payload_tilt_deg', 'ring tilt', 'deg', '{:.1f} deg', 5.0),
                                               (axZ, 'payload_z', 'ring height', 'm', '{:.2f} m', 1.2)):
            v = run[col].to_numpy(float)[w]
            ax.plot(self.tw, v, color='#D6D9DD', lw=2.0, zorder=2)
            top = max(floor, 1.12 * np.nanmax(v)) if np.isfinite(v).any() else floor
            ax.set_ylim(0, top)
            ax.set_xlim(t0, t1)
            ax.grid(True, color='#E9ECEF', lw=0.8)
            for sp in ('top', 'right'):
                ax.spines[sp].set_visible(False)
            ax.tick_params(labelsize=13, colors=INK)
            ax.set_ylabel(unit, fontsize=14, color=GREY)
            ax.text(0.0, 1.0, name, transform=ax.transAxes, ha='left', va='bottom', fontsize=17,
                    color=INK, fontweight='bold', bbox=dict(boxstyle='square,pad=0.1', fc='white', ec='none'))
            for g in self.groups:
                ax.axvline(g['t'], color=g['colour'], lw=1.3, alpha=0.75, zorder=1)
            past = self._add(ax.plot([], [], color=RING, lw=2.6)[0], 3)
            dot = self._add(ax.plot([], [], 'o', ms=8, color=INK)[0], 5)
            cur = self._add(ax.axvline(t0, color=INK, lw=1.4), 4)
            val = self._add(ax.text(1.0, 1.0, '', transform=ax.transAxes, ha='right', va='bottom',
                                    fontsize=17, color=INK, family='DejaVu Sans Mono',
                                    bbox=dict(boxstyle='square,pad=0.1', fc='white', ec='none')), 6)
            self.series.append(dict(v=v, past=past, dot=dot, cur=cur, val=val, fmt=fmt))
        plt.setp(axT.get_xticklabels(), visible=False)
        axZ.set_xlabel('sim time (s)', fontsize=14, color=GREY)
        self._labels()

    def _labels(self):
        """Event labels above the tilt panel: horizontal, greedy rows, a leader down to the axes."""
        fig, ax = self.fig, self.axT
        r = fig.canvas.get_renderer()
        top = ax.get_window_extent(r).y1 + 30          # clear of the panel's name
        lo, hi = 1340, W_PX - 12
        rows = []
        for g in self.groups:
            xp = ax.transData.transform((g['t'], 0))[0]
            txt = fig.text(0, 0, g['label'], fontsize=13, color=g['colour'], ha='center', va='bottom')
            wd = txt.get_window_extent(r).width
            xc = min(max(xp, lo + wd / 2), hi - wd / 2)
            span = (xc - wd / 2 - 8, xc + wd / 2 + 8)
            row = next((k for k, used in enumerate(rows) if all(span[1] < a or span[0] > b for a, b in used)), None)
            if row is None:
                if len(rows) < 8:
                    rows.append([])
                    row = len(rows) - 1
                else:
                    row = len(rows) - 1
            rows[row].append(span)
            y = top + 4 + 25 * row
            txt.set_position((fx(xc), fy(y)))
            fig.add_artist(Line2D([fx(xp)] * 2, [fy(top - 30), fy(y - 1)], transform=fig.transFigure,
                                  color=g['colour'], lw=0.9, alpha=0.75, zorder=-1))   # under the panel's text

    # -- text
    def _text(self, title):
        f, S = self.fig, self.S
        self.t_txt = self._add(f.text(fx(28), fy(H_PX - 22), '', ha='left', va='top', fontsize=30, color=INK,
                                      family='DejaVu Sans Mono'), 10)
        f.text(fx(30), fy(H_PX - 74), speed_words(self.speed), ha='left', va='top', fontsize=15, color=GREY)
        self.fleet_txt = self._add(f.text(fx(30), fy(H_PX - 102), '', ha='left', va='top', fontsize=19,
                                          color=INK), 10)
        hc = ps.drone_colour(S['hi']) if S['hi'] is not None else INK
        self.phase_txt = self._add(f.text(fx(30), fy(H_PX - 134), '', ha='left', va='top', fontsize=19,
                                          color=hc), 10)
        self.banner = self._add(f.text(fx(850), fy(H_PX - 40), '', ha='center', va='top', fontsize=30,
                                       fontweight='bold',
                                       bbox=dict(boxstyle='round,pad=0.35', fc='white', ec='none', alpha=0.85)),
                                11)
        f.add_artist(Line2D([0, 1], [fy(64), fy(64)], transform=f.transFigure, color='#D6D9DD', lw=1.0))
        stamp = f"{S['id']} . Gazebo twin" + (f" . {S['cfg']}" if S['cfg'] else '')
        st = f.text(fx(W_PX - 20), fy(32), stamp.replace(' . ', '  ·  '), ha='right', va='center',
                    fontsize=12, color=GREY)
        cap = f.text(fx(28), fy(32), title, ha='left', va='center', fontsize=21, color=INK)
        r = f.canvas.get_renderer()
        room = st.get_window_extent(r).x0 - 30            # the caption shrinks to stay clear of the stamp
        while cap.get_window_extent(r).x1 > room and cap.get_fontsize() > 14:
            cap.set_fontsize(cap.get_fontsize() - 0.5)

    # -- one frame
    def update(self, k):
        S, tr, cam = self.S, self.tr, self.cam
        t = tr.tf[k]
        P, R = tr.P[k], tr.R[k]
        ring = P + self.circ @ R.T
        sx, sy = cam.xy(ring)
        dep = cam.depth(ring - P)
        far = dep <= 0
        far = far | np.roll(far, 1) | np.roll(far, -1)
        near = dep >= 0
        near = near | np.roll(near, 1) | np.roll(near, -1)
        self.ring_far.set_data(np.where(far, sx, np.nan), np.where(far, sy, np.nan))
        self.ring_near.set_data(np.where(near, sx, np.nan), np.where(near, sy, np.nan))
        self.ring_fill.set_xy(np.column_stack([sx, sy]))
        flat = ring.copy()
        flat[:, 2] = 0.0
        self.sh_ring.set_data(*cam.xy(flat))

        pl = plates(S, P, R)
        shadows, order = [], []
        for i in range(S['n_all']):
            p = tr.D[i][k]
            if not np.all(np.isfinite(p)):
                for a in (self.rods[i], self.arms[i], self.props[i], self.hubs[i]):
                    a.set_data([], [])
                continue
            shadows.append([p[0], p[1], 0.0])
            on = tr.armed[i][k]
            px, py = cam.xy(p)
            self.arms[i].set_data(self.quad[0][:, 0] + px, self.quad[0][:, 1] + py)
            self.props[i].set_data(self.quad[1][:, 0] + px, self.quad[1][:, 1] + py)
            self.arms[i].set_alpha(1.0 if on else 0.35)
            self.props[i].set_alpha(1.0 if on else 0.35)
            self.hubs[i].set_data([px], [py])
            self.hubs[i].set_markerfacecolor(self.colours[i] if on else 'white')
            pivot = p + [0.0, 0.0, S['pivot']]
            end = None
            if i < S['n_car'] and t < S['released'].get(i, math.inf):
                end = pl[i]
            elif i >= S['n_car'] and i == S['hi'] and tr.tip is not None and np.all(np.isfinite(tr.tip[k])):
                end = tr.tip[k]
            if end is None:
                self.rods[i].set_data([], [])
            else:
                self.rods[i].set_data(*cam.xy(np.array([pivot, end])))
                self.rods[i].set_markevery([1] if i < S['n_car'] else [])
            order.append((cam.depth(p), i))
        if shadows:
            self.sh_drones.set_data(*cam.xy(np.array(shadows)))
        else:
            self.sh_drones.set_data([], [])

        if S['plate_new'] is not None:
            pn = P + R @ np.array([S['plate_new'][0], S['plate_new'][1], S['z_att']])
            self.plate_new.set_data(*cam.xy(pn[None, :]))
        if tr.tip is not None and np.all(np.isfinite(tr.tip[k])):
            self.tip.set_data(*cam.xy(tr.tip[k][None, :]))
        else:
            self.tip.set_data([], [])
        b = tr.ball[k] if tr.ball is not None else None
        if b is not None and np.all(np.isfinite(b)) and np.all(b[:2] >= self.box[0] - 1) \
                and np.all(b[:2] <= self.box[1] + 1):
            self.ball.set_data(*cam.xy(b[None, :]))
        else:
            self.ball.set_data([], [])

        # the drones far to near, each above its own rod
        for z, (_, i) in enumerate(sorted(order)):
            self.rods[i].set_zorder(4 + 0.01 * z)
            self.arms[i].set_zorder(7 + 0.01 * z)
            self.props[i].set_zorder(7.002 + 0.01 * z)
            self.hubs[i].set_zorder(7.005 + 0.01 * z)

        self.t_txt.set_text(f't = {t:6.1f} s')
        fl = [w for te, w in self.fleet if te <= t]
        self.fleet_txt.set_text('fleet: ' + (fl[-1] if fl else 'on the floor, disarmed'))
        ph = [w for te, w in self.phases if te <= t]
        self.phase_txt.set_text(("Tejen's phase: " + ph[-1]) if (S['partner'] and ph) else '')
        ban = [g for g in self.groups if g['t'] <= t and (t - g['t']) / self.speed < FLASH_S]
        if ban:
            g = ban[-1]
            age = (t - g['t']) / self.speed
            self.banner.set_text(g['label'])
            self.banner.set_color(g['colour'])
            self.banner.set_alpha(min(1.0, 2.0 * (FLASH_S - age)))
            self.banner.get_bbox_patch().set_alpha(0.85 * min(1.0, 2.0 * (FLASH_S - age)))
        else:
            self.banner.set_text('')

        j = int(np.searchsorted(self.tw, t, 'right'))
        for s in self.series:
            v_now = float(np.interp(t, self.tw, s['v'])) if len(self.tw) else np.nan
            s['past'].set_data(np.r_[self.tw[:j], t], np.r_[s['v'][:j], v_now])
            s['dot'].set_data([t], [v_now])
            s['cur'].set_xdata([t, t])
            s['val'].set_text(s['fmt'].format(v_now) if np.isfinite(v_now) else '')

    def frame(self, k):
        """Draw frame k; returns the canvas buffer (H, W, 4) uint8 RGBA."""
        self.update(k)
        canvas = self.fig.canvas
        canvas.restore_region(self.bg)
        for a in sorted(self.anim, key=lambda a: a.get_zorder()):
            self.fig.draw_artist(a)
        return canvas.buffer_rgba()


def _mix(c1, c2, f):
    from matplotlib.colors import to_rgb
    a, b = np.array(to_rgb(c1)), np.array(to_rgb(c2))
    return tuple((1 - f) * a + f * b)


# -- main ------------------------------------------------------------------------------------------------

def frame_times(t0, t1, speed, fps, decimate=1):
    """Sim times of the video's frames: one every speed*decimate/fps sim seconds from t0 to t1."""
    step = speed * decimate / fps
    n = int(math.floor((t1 - t0) / step + 1e-9)) + 1
    return t0 + step * np.arange(n)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run', help='run id (R1487) or run directory')
    ap.add_argument('--out', help='mp4 path (default results/videos/<run dir name>.mp4)')
    ap.add_argument('--speed', type=float, default=1.0, help='sim seconds per video second')
    ap.add_argument('--t0', default='start', help="window start: seconds or EVENT[#n][+-s] (default start)")
    ap.add_argument('--t1', default='end', help="window end: seconds or EVENT[#n][+-s] (default end)")
    ap.add_argument('--title', help='the caption line: what the run shows, in plain words')
    ap.add_argument('--fps', type=float, default=30.0)
    ap.add_argument('--decimate', type=int, default=1, help='render every Nth frame (output at fps/N)')
    ap.add_argument('--view', default=f'{VIEW[0]},{VIEW[1]}', help='camera azimuth,elevation (deg)')
    ap.add_argument('--still', type=str, nargs='+', help='write PNGs at these sim times instead of a video')
    ap.add_argument('--ffmpeg', default='/usr/bin/ffmpeg' if os.path.exists('/usr/bin/ffmpeg') else 'ffmpeg')
    a = ap.parse_args(argv)

    S = load(a.run)
    t_first, t_last = float(S['t'][0]), float(S['t'][-1])
    t0 = max(t_first, when(a.t0, S['events'], t_first, t_last))
    t1 = min(t_last, when(a.t1, S['events'], t_first, t_last))
    if t1 <= t0:
        raise SystemExit(f'empty window {t0:.2f}..{t1:.2f} s')
    title = a.title or f"Twin run {S['id']}" + (f" ({S['cfg']})" if S['cfg'] else '')
    view = tuple(float(v) for v in a.view.split(','))
    out = a.out or os.path.join(os.path.dirname(TOOLS), 'results', 'videos', os.path.basename(S['dir']) + '.mp4')
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    if a.still:
        ts = np.array([when(s, S['events'], t_first, t_last) for s in a.still])
        v = Video(S, t0, t1, np.clip(ts, t0, t1), a.speed, title, view)
        stem = os.path.splitext(out)[0]
        for k, ts_k in enumerate(v.tr.tf):
            p = f'{stem}_t{ts_k:07.2f}.png'
            plt.imsave(p, np.asarray(v.frame(k)))
            print(p)
        return 0

    tf = frame_times(t0, t1, a.speed, a.fps, a.decimate)
    fps_out = a.fps / a.decimate
    v = Video(S, t0, t1, tf, a.speed, title, view)
    print(f"{S['id']}: {t0:.2f}..{t1:.2f} s sim at {a.speed:g}x -> {len(tf)} frames, "
          f"{len(tf) / fps_out:.1f} s of video at {fps_out:g} fps -> {out}", flush=True)
    cmd = [a.ffmpeg, '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo', '-pix_fmt', 'rgba',
           '-s', f'{W_PX}x{H_PX}', '-r', f'{fps_out:g}', '-i', '-', '-c:v', 'libx264', '-preset', 'medium',
           '-crf', '20', '-pix_fmt', 'yuv420p', '-threads', '2', '-movflags', '+faststart', out]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    tic, mark = time.time(), 0
    try:
        for k in range(len(tf)):
            p.stdin.write(v.frame(k))
            if k * 10 // len(tf) > mark:
                mark = k * 10 // len(tf)
                el = time.time() - tic
                print(f'  {100 * k / len(tf):3.0f} %  {el:6.0f} s, ~{el / k * (len(tf) - k):.0f} s left', flush=True)
    finally:
        p.stdin.close()
        rc = p.wait()
    if rc:
        raise SystemExit(f'ffmpeg failed ({rc})')
    print(f'{out}: {len(tf)} frames in {time.time() - tic:.0f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
