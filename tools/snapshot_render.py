#!/usr/bin/env python3
"""
tools/snapshot_render.py -- vector 3D snapshots of a simulation run, rebuilt from its logged poses (no Gazebo).

    python3 tools/snapshot_render.py results/<date>/R####_<name> --times 44,100,140,340 --out /tmp/shots
    python3 tools/snapshot_render.py <run_dir> --times 44,100 --azim -60 --elev 28 --out <stem>

One orthographic camera for every panel, on white: the ring black (its far half lighter), its plates as small dots,
each UAV a small disc coloured by its role (carrying blue, rejoining red, departed a hollow grey disc on a dashed
cable), each cable a thin line from the UAV's pivot to its end, a plate hollow once its carrier has left, the ring's reference path light grey, a 1 m scale bar. The panel label carries the time and the
number of carriers. Library use (the storyboards): `Run(run_dir)`, `Camera(azim, elev)`, `draw_snapshot(ax, run, t,
cam)`, `roles_at(run, t)`.

Data: <run>/logs/run.csv (ring pose, UAV positions, `d<i>_attached`), logs/events.csv (REJOIN_START / REJOIN_WELD for
the rejoining role), the planner's params.json (attach radius, azimuths -- counter-clockwise inside --, attach_z), and
gz_record/state.tlog when the run was recorded: the rods' logged poses, so a free cable is drawn where it hung. Without
the state log a carrying cable is drawn from the UAV's pivot to its plate and a free one straight down.
run.csv's t is the run clock; the state log's is the simulation clock, t + t_ready_sim (record.json).
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sqlite3
import sys

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

PIVOT_DZ = -0.04                 # the cable hangs from a joint 4 cm under the body (rig geometry, every twin world)
UAV_R = 0.10                     # m: the disc drawn for a UAV (about the rotor span)
ROLE_COLOUR = {'carrying': '#2166ac', 'departed': '0.45', 'rejoining': '#b2182b'}
ROLE_ORDER = ('carrying', 'rejoining', 'departed')
_NAME = -998690179357215250      # gz-sim component type ids (Name, Pose) in a SerializedStateMap
_POSE = -7527930132038368260


def _rot(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _pose_matrix(p):
    """gz Pose text 'x y z roll pitch yaw' (R = Rz Ry Rx) as a 4x4."""
    x, y, z, r, pi, ya = p
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(pi), math.sin(pi), math.cos(ya), math.sin(ya)
    m = np.eye(4)
    m[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    m[:3, 3] = (x, y, z)
    return m


def read_rods(tlog, n):
    """Each tether's rod over the state log: times (s, simulation clock) and the rod's two ends in the world,
    top (UAV side) and end (plate side), shapes (T,) and (T, n, 3). None when the log has no tethers."""
    from gz.msgs10 import serialized_map_pb2 as sm
    from gz.msgs10 import stringmsg_pb2 as smsg
    db = sqlite3.connect(f'file:{tlog}?mode=ro', uri=True)
    sdf = smsg.StringMsg()
    sdf.ParseFromString(db.execute('select message from messages where topic_id=1 order by id limit 1').fetchone()[0])
    first = None
    for (msg,) in db.execute('select message from messages where topic_id=2 order by id limit 1'):
        first = sm.SerializedStateMap()
        first.ParseFromString(msg)
    names = {eid: e.components[_NAME].component.decode() for eid, e in first.entities.items() if _NAME in e.components}
    model, link, half = {}, {}, {}
    for i in range(n):
        ids = [eid for eid, nm in names.items() if nm == f'tether_{i}']
        if not ids:
            return None
        model[i] = ids[0]
        link[i] = min(eid for eid, nm in names.items() if nm == 'rod' and eid > ids[0])
        blk = sdf.data[sdf.data.find(f"<model name='tether_{i}'>"):]
        m = re.search(r'<cylinder>\s*<radius>[^<]*</radius>\s*<length>([^<]+)</length>', blk)
        half[i] = 0.5 * float(m.group(1)) if m else 0.5
    want = set(model.values()) | set(link.values())
    cur, times, tops, ends = {}, [], [], []
    for tr, msg in db.execute('select time_recv, message from messages where topic_id=2 order by id'):
        s = sm.SerializedStateMap()
        s.ParseFromString(msg)
        for eid in want & set(s.entities):
            c = s.entities[eid].components
            if _POSE in c:
                cur[eid] = _pose_matrix([float(v) for v in c[_POSE].component.decode().split()])
        if len(cur) < len(want):
            continue
        times.append(tr * 1e-9)
        w = [cur[model[i]] @ cur[link[i]] for i in range(n)]
        tops.append([(w[i] @ [0, 0, half[i], 1])[:3] for i in range(n)])
        ends.append([(w[i] @ [0, 0, -half[i], 1])[:3] for i in range(n)])
    return np.array(times), np.array(tops), np.array(ends)


class Run:
    """A simulation run's logged poses on the run clock."""

    def __init__(self, run_dir, rods=True):
        self.dir = run_dir
        self.name = os.path.basename(os.path.normpath(run_dir))
        df = pd.read_csv(os.path.join(run_dir, 'logs', 'run.csv')).drop_duplicates('t').sort_values('t')
        self.df = df.reset_index(drop=True)
        self.t = self.df.t.to_numpy(float)
        self.n = len([c for c in df.columns if re.fullmatch(r'd\d+_x', c)])
        self.events = []
        self.joints = {}            # i -> [(t, welded)]: the welds and releases (a weld at load time has no time)
        for e in csv.DictReader(open(os.path.join(run_dir, 'logs', 'events.csv'))):
            t = float(e['sim_time']) if e['sim_time'] not in ('', 'nan') else -math.inf
            if e['event'] in ('JOINT_ATTACHED', 'JOINT_DETACHED') and e['arg'].strip().isdigit():
                self.joints.setdefault(int(e['arg']), []).append((t, e['event'] == 'JOINT_ATTACHED'))
            if math.isfinite(t):
                self.events.append((t, e['event'], e['arg']))
        (pj,) = glob.glob(os.path.join(run_dir, 'logs', 'dissipative_planner', '*', 'params.json'))[:1]
        p = json.load(open(pj))
        az = [math.radians(float(a)) for a in str(p['attach_azimuths_deg']).split(',')]
        r, z = float(p['attach_radius']), float(p['attach_z'])
        self.plates = np.array([[r * math.cos(a), r * math.sin(a), z] for a in az])      # ring frame, drone order
        self.ring_r = r
        rec = os.path.join(run_dir, 'record.json')
        self.t_ready = json.load(open(rec)).get('t_ready_sim') if os.path.exists(rec) else None
        tlog = os.path.join(run_dir, 'gz_record', 'state.tlog')
        self.rods = read_rods(tlog, self.n) if rods and os.path.exists(tlog) and self.t_ready is not None else None

    def event_times(self, name, arg=None):
        return [t for t, e, a in self.events if e == name and (arg is None or a == str(arg))]

    def _row(self, t):
        return self.df.iloc[int(np.clip(np.searchsorted(self.t, t), 0, len(self.t) - 1))]

    def ring(self, t):
        r = self._row(t)
        return (np.array([r.payload_x, r.payload_y, r.payload_z], float),
                _rot([r.payload_qw, r.payload_qx, r.payload_qy, r.payload_qz]))

    def uav(self, t, i):
        r = self._row(t)
        return np.array([r[f'd{i}_x'], r[f'd{i}_y'], r[f'd{i}_z']], float)

    def attached(self, t, i):
        """Welded at run time t: the joint events (events.csv) when the run has them, else run.csv's column."""
        if self.joints:
            ev = sorted(self.joints.get(i, []))
            past = [w for te, w in ev if te <= t]
            # before its first event a UAV is as the world loaded it: welded, unless that event is a timed weld
            return bool(past[-1]) if past else not (ev and ev[0][1] and math.isfinite(ev[0][0]))
        return bool(self._row(t)[f'd{i}_attached'] > 0.5)

    def plate_world(self, t, i):
        c, rm = self.ring(t)
        return c + rm @ self.plates[i]

    def cable(self, t, i):
        """(pivot end, plate end) of UAV i's cable at run time t."""
        if self.rods is not None:
            ts, tops, ends = self.rods
            k = int(np.clip(np.searchsorted(ts, t + self.t_ready), 0, len(ts) - 1))
            return tops[k, i], ends[k, i]
        top = self.uav(t, i) + [0.0, 0.0, PIVOT_DZ]
        return top, (self.plate_world(t, i) if self.attached(t, i) else top - [0.0, 0.0, 1.0])

    def cable_end_series(self, i):
        """Run times and UAV i's cable end in the world, from the state log (None without one)."""
        if self.rods is None:
            return None
        ts, _, ends = self.rods
        return ts - self.t_ready, ends[:, i]


def roles_at(run, t):
    """'carrying' when welded, 'rejoining' from its REJOIN_START to its REJOIN_WELD, else 'departed'."""
    out = []
    for i in range(run.n):
        if run.attached(t, i):
            out.append('carrying')
            continue
        starts = [s for s in run.event_times('REJOIN_START', i) if s <= t]
        welds = [w for w in run.event_times('REJOIN_WELD', i) if starts and w >= starts[-1]]
        out.append('rejoining' if starts and (not welds or t < welds[0]) else 'departed')
    return out


class Camera:
    """Orthographic camera looking at the scene from azimuth `azim` (deg, from +x, counter-clockwise) and
    elevation `elev` (deg above the horizon)."""

    def __init__(self, azim=-55.0, elev=30.0):
        a, e = math.radians(azim), math.radians(elev)
        self.d = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])   # toward the viewer
        self.r = np.array([-math.sin(a), math.cos(a), 0.0])
        self.u = np.cross(self.d, self.r)

    def __call__(self, p):
        p = np.atleast_2d(np.asarray(p, float))
        return np.c_[p @ self.r, p @ self.u]

    def depth(self, p):
        return np.atleast_2d(np.asarray(p, float)) @ self.d


def _circle(c, rm, radius, k=120):
    a = np.linspace(0, 2 * np.pi, k + 1)
    return c + (rm @ np.c_[radius * np.cos(a), radius * np.sin(a), np.zeros_like(a)].T).T


def draw_snapshot(ax, run, t, cam, roles=None, path=None, ring_lw=1.6, cable_lw=0.6, uav_lw=0.5, colours=None,
                  emphasise=None):
    """One frame at run time t on `ax` (equal aspect is the caller's). `path` is an (N, 3) array drawn light grey
    under everything; `emphasise` lists UAVs drawn larger (a storyboard's subject)."""
    colours = colours or ROLE_COLOUR
    roles = roles or roles_at(run, t)
    c, rm = run.ring(t)
    if path is not None:
        uv = cam(path)
        ax.plot(uv[:, 0], uv[:, 1], color='0.82', lw=0.7, zorder=1, solid_capstyle='round')
    ring = _circle(c, rm, run.ring_r)
    uv, dep = cam(ring), cam.depth(ring) - cam.depth(c)[0]
    far = dep < 0
    for mask, col, z in ((far, '0.55', 2), (~far, 'black', 6)):
        seg = np.where(mask, 1.0, np.nan)
        ax.plot(uv[:, 0] * seg, uv[:, 1] * seg, color=col, lw=ring_lw, zorder=z, solid_capstyle='round')
    # plates: filled where a carrier hangs, hollow where its carrier has left
    pl = cam(c + (rm @ run.plates.T).T)
    held = np.array([r == 'carrying' for r in roles])
    ax.plot(pl[held, 0], pl[held, 1], 'o', ms=2.2, color='black', zorder=7)
    ax.plot(pl[~held, 0], pl[~held, 1], 'o', ms=2.4, mfc='white', mec='black', mew=0.6, zorder=7)
    for i in range(run.n):
        top, end = run.cable(t, i)
        col = colours[roles[i]]
        free = roles[i] == 'departed'
        seg = cam(np.vstack([top, end]))
        ax.plot(seg[:, 0], seg[:, 1], color='0.62' if free else '0.35', lw=cable_lw, zorder=4,
                ls=(0, (2, 1.5)) if free else '-')
        big = emphasise is not None and i in emphasise
        disc = cam(_circle(run.uav(t, i), np.eye(3), UAV_R * (1.25 if big else 1.0), 40))
        z = 8 + float(cam.depth(run.uav(t, i))[0])          # nearer UAVs over farther ones
        if free:                                            # a departed UAV: hollow, so grey means only 'off the ring'
            ax.fill(disc[:, 0], disc[:, 1], facecolor='white', edgecolor=col, lw=0.9, zorder=z)
        else:
            ax.fill(disc[:, 0], disc[:, 1], facecolor=col, edgecolor='white', lw=uav_lw, zorder=z)
    return roles


def scale_bar(ax, length=1.0, text='1 m', at=(0.05, 0.07)):
    """A `length` bar along the image's horizontal (true length in an orthographic view with equal aspect), its left
    end at `at` (axes fraction); call after the limits are set."""
    xl, yl = ax.get_xlim(), ax.get_ylim()
    x0, y0 = xl[0] + at[0] * (xl[1] - xl[0]), yl[0] + at[1] * (yl[1] - yl[0])
    ax.plot([x0, x0 + length], [y0, y0], color='0.3', lw=1.0, solid_capstyle='butt', zorder=9)
    ax.annotate(text, xy=(x0 + length / 2, y0), xytext=(0, 2), textcoords='offset points', ha='center',
                va='bottom', fontsize=8, color='0.3')


def extent(run, times, cam, path=None, pad=0.06):
    """Shared axis limits that hold every UAV, cable and the ring at `times` (and the path)."""
    pts = []
    for t in times:
        c, rm = run.ring(t)
        pts.append(_circle(c, rm, run.ring_r, 24))
        for i in range(run.n):
            pts.extend([np.atleast_2d(p) for p in run.cable(t, i)])
            pts.append(np.atleast_2d(run.uav(t, i)) + [[UAV_R, 0, 0], [-UAV_R, 0, 0], [0, UAV_R, 0], [0, -UAV_R, 0]])
    if path is not None:
        pts.append(path)
    uv = cam(np.vstack(pts))
    lo, hi = uv.min(0), uv.max(0)
    span = hi - lo
    return (lo[0] - pad * span[0], hi[0] + pad * span[0]), (lo[1] - pad * span[1], hi[1] + pad * span[1])


def reference_path(run, t0=None, t1=None):
    """The ring's reference over [t0, t1] (default: the whole run), at its hover height (within 2 cm of the median)."""
    d = run.df
    m = d.payload_ref_z > 0.3
    if t0 is not None:
        m &= d.t >= t0
    if t1 is not None:
        m &= d.t <= t1
    p = d.loc[m, ['payload_ref_x', 'payload_ref_y', 'payload_ref_z']].to_numpy(float)
    # the path at its height only: the climb to it and the descent from it are not the path
    return p[abs(p[:, 2] - np.median(p[:, 2])) < 0.02] if len(p) else p


def main():
    import matplotlib.pyplot as plt
    import plot_style as ps
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run_dir')
    ap.add_argument('--times', required=True, help='run-clock seconds, comma separated (one panel each)')
    ap.add_argument('--out', required=True, help='output stem (writes <stem>.pdf and <stem>.png)')
    ap.add_argument('--azim', type=float, default=-55.0)
    ap.add_argument('--elev', type=float, default=30.0)
    ap.add_argument('--cols', type=int, default=2)
    ap.add_argument('--width', type=float, default=ps.TEXTWIDTH_IN)
    a = ap.parse_args()
    times = [float(x) for x in a.times.split(',')]
    run, cam = Run(a.run_dir), Camera(a.azim, a.elev)
    path = reference_path(run)
    xl, yl = extent(run, times, cam, path)
    ps.thesis()
    rows = math.ceil(len(times) / a.cols)
    w_ax = a.width / a.cols
    fig, axes = plt.subplots(rows, a.cols, figsize=(a.width, rows * w_ax * (yl[1] - yl[0]) / (xl[1] - xl[0]) + 0.5),
                             squeeze=False)
    for k, (ax, t) in enumerate(zip(axes.flat, times)):
        roles = draw_snapshot(ax, run, t, cam, path=path)
        ax.set_xlim(*xl)
        ax.set_ylim(*yl)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        ps.panel_label(ax, 'abcdefgh'[k])
        ax.annotate(f'{roles.count("carrying")} carriers, $t = {t:.0f}$\\,s', xy=(0, 1), xycoords='axes fraction',
                    xytext=(22, 4), textcoords='offset points', ha='left', va='bottom', fontsize=9)
    scale_bar(axes.flat[0])
    for ax in list(axes.flat)[len(times):]:
        ax.set_visible(False)
    ps.save_print(fig, a.out)
    print(a.out + '.pdf')


if __name__ == '__main__':
    main()
