#!/usr/bin/env python3
"""Every real-world descent of QUAD4's magnet toward its target on Wed 4 and Thu 5 Nov 2026: one row each.

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    nice -n 10 python3 results/analysis/rig_joins_2026-11-04_05/joins.py          # -> joins.csv, flights.csv
    python3 results/analysis/rig_joins_2026-11-04_05/summarise.py                  # -> summary.md

Exploratory reading of recorded flights (no bars). Per flight: the launch line (launches.csv), the full launch log
(his planner's phases and our magnet manager's CONTACT / PROOF lines) and the bag (mocap of the ring, QUAD4's body,
the magnet tip marker, the pickup object; his reference).

A descent is each entry into DESCEND_TO_ATTACHMENT (from MATCH_VELOCITY, or from LIFT_PROOF after a failed proof)
or DESCEND_TO_PICKUP; it ends when his planner leaves the descent and its bottom hold (ATTACH_READY,
MAGNET_ATTACH_WAIT). A drop is each DROP_OBJECT. Geometry, per flight:
- plate target: the weld point the launch typed (contact_target_xy / _z, ring frame: the tip marker's position when
  welded on that plate; plates 1 and 8 are plate 9's point turned, as the launch does). Height = along the ring's
  normal above the weld point (the full ring attitude, as our contact check: the bag reproduces every CONTACT line).
  Horizontal offsets = along the plate's radius projected on the floor (+ outward) and anticlockwise seen from above
  (+), so QUAD4's body 1 m up is not thrown sideways when the ring leans.
- 5 Nov fm9y_2 typed 4 Nov's QUAD4 values (W 0.0365, X 0.0039, Y 0.2271) after body 24 was redefined on 4 Nov;
  it is scored against 5 Nov's measured point (W 0.0491, X 0, Y 0.2293), the one every other 5 Nov flight typed and
  its own welded start confirms (1.2 mm, +0.5 mm).
- "at plate height": the first tip sample at or below the weld point's height, or the lowest sample if the magnet
  settled within PLATE_BAND above it; the offsets are read there. A descent that never got within PLATE_BAND is read
  at its lowest tip sample. The lowest sample of every descent is also given (low_* columns).
- pickup target: the object's Motive body (/pickup/motion_capture_state) xy; height gap = tip z - object z - the typed
  pickup_contact_height, read the same way. Offsets in the world frame (x, y).
- drop: the tip against the ring centre at DROP_OBJECT, and the object's first impact (its fall relative to the ring
  stops) against the ring centre.

Bags are read with sqlite + rclpy deserialisation (~15 min for both days on the laptop, niced). JOINS_CACHE=<dir>
keeps a pickle per bag for re-runs; JOINS_OUT=<dir> writes the CSVs elsewhere; flight names as arguments
(e.g. 2026-11-05/sq8_2) limit the run (for testing; the CSVs then hold only those flights).
"""
import csv
import glob
import math
import os
import re
import sqlite3
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

REPO = os.path.expanduser('~/multi_drone_control')
RIG = os.path.join(REPO, 'results/rig')
OUT = os.environ.get('JOINS_OUT', os.path.dirname(os.path.abspath(__file__)))
DAYS = ('2026-11-04', '2026-11-05')
PLATE_BAND = 0.010          # m above the weld point (or the object's contact height) that counts as "at plate height"
CONTACT_K = 0.025           # m, our contact radius (comotion contact xy), for the "inside K" column
WELD_5NOV = (0.0, 0.2293, 0.0491)   # body 24 as redefined 4 Nov ~17:50 (the 11 Nov sheet's X, Y, W)
TOPICS = {'tip': '/magnet_tip_pose', 'body': '/drone_3/motion_capture_state', 'ring': '/payload/motion_capture_state',
          'obj': '/pickup/motion_capture_state', 'phase': '/join_planner/phase', 'ref': '/join_planner/reference'}
STAMP = r'\[(\d{9,}\.\d+)\]'


# ----------------------------------------------------------------------------------------------- launch line and log
def launches():
    out = []
    for day in DAYS:
        with open(os.path.join(RIG, day, 'launches.csv')) as f:
            for r in csv.DictReader(f):
                args = {}
                for k, v in re.findall(r'(\S+?):=(\S+)', r['launch'].replace('\n', ' ')):
                    args.setdefault(k, v)
                out.append({'day': day, 'name': r['name'], 'time': r['time'], 'armed': r['armed'], 'flew': r['flew'],
                            'args': args})
    return out


def registry():
    rows = {}
    with open(os.path.join(REPO, 'results/registry.csv'), newline='') as f:
        for r in csv.reader(f):
            if r and r[0].startswith(('RIG-1104-', 'RIG-1105-')):
                rows[r[0]] = r
    return rows


def plate_of(args):
    p = args.get('reattach_plate') or args.get('plate') or '9'
    return int(float(p))


def turned(pt9, plate):
    """Plate 9's ring-frame point turned to `plate` (clockwise numbering; maths counter-clockwise)."""
    a = math.radians(-(30.0 * plate - 270.0))
    c, s = math.cos(a), math.sin(a)
    return (c * pt9[0] - s * pt9[1], s * pt9[0] + c * pt9[1])


def parse_log(path):
    ev = {'phases': [], 'contacts': [], 'proof_failed': [], 'proven': [], 'release': [], 'resize4': [],
          'his_takeoff': [], 'pick_attached': [], 'pick_failed': [], 'lifted': [], 'drop': [], 'held': [],
          'payload_dropped': [], 'liftproof': [], 'contact_target': None, 'geometry': None, 'irl_attached': []}
    if not os.path.exists(path):
        return ev
    with open(path, errors='replace') as f:
        for ln in f:
            m = re.search(STAMP, ln)
            t = float(m.group(1)) if m else None
            if ev['contact_target'] is None:
                g = re.search(r'contact_target_z:=([-\d.]+) contact_target_xy:=([-\d.]+),([-\d.]+)', ln)
                if g:
                    ev['contact_target'] = (float(g.group(2)), float(g.group(3)), float(g.group(1)))
            if ev['geometry'] is None:
                g = re.search(r'geometry: n=\d+ cable_len=[\d.]+ attach_radius=([\d.]+) attach_z=[\d.]+ load_mass=([\d.]+)', ln)
                if g:
                    ev['geometry'] = (float(g.group(1)), float(g.group(2)))
            if t is None:
                continue
            g = re.search(r'\[online_join_planner\]: Planner phase: (\w+) -> (\w+)\. Reason: (.*)$', ln)
            if g:
                ev['phases'].append((t, g.group(1), g.group(2), g.group(3).strip()))
                if g.group(1) == 'WAIT_FOR_TAKEOFF' and g.group(2) == 'TAKEOFF':
                    ev['his_takeoff'].append(t)
            g = re.search(r'CONTACT \(try (\d+)\): xy ([\d.]+) mm, normal ([-+\d.]+) mm', ln)
            if g:
                ev['contacts'].append((t, int(g.group(1)), float(g.group(2)), float(g.group(3))))
                continue
            g = re.search(r'PROOF FAILED \(try (\d+), (\w+)\): drift ([\d.]+) mm, plate rise ([-\d.]+) mm', ln)
            if g:
                ev['proof_failed'].append((t, int(g.group(1)), g.group(2), float(g.group(3)), float(g.group(4))))
                continue
            g = re.search(r'ATTACH: proven \(try (\d+)\).*?drift ([\d.]+) mm', ln)
            if g:
                ev['proven'].append((t, int(g.group(1)), float(g.group(2))))
                continue
            if 'PARTNER RELEASE' in ln:
                ev['release'].append(t)
            elif 'FLEET RESIZED to n=4' in ln:
                ev['resize4'].append(t)
            elif 'PAYLOAD DROPPED' in ln:
                ev['payload_dropped'].append(t)
            elif 'Contact descent held' in ln:
                ev['held'].append(t)
            elif 'ASSUMED ATTACHED' in ln:
                ev['irl_attached'].append(t)
            g = re.search(r'ATTACH drone 3 \(start weld[^)]*\): n=\d+, plate at ([\d.]+) deg cw', ln)
            if g and ev.get('start_plate_deg') is None:
                ev['start_plate_deg'] = float(g.group(1))
            g = re.search(r'object attached with xy error ([\d.]+) m', ln)
            if g:
                ev['pick_attached'].append((t, float(g.group(1))))
            g = re.search(r'pickup attempt (\d+) failed: (.*?);', ln)
            if g:
                tip = re.search(r'the tip ([\d.]+) m off target', g.group(2))
                ev['pick_failed'].append((t, int(g.group(1)), g.group(2), float(tip.group(1)) if tip else math.nan))
            g = re.search(r'object lifted ([\d.]+) m', ln)
            if g:
                ev['lifted'].append((t, float(g.group(1))))
            g = re.search(r'drop on arrival: the magnet ([\d.]+) m from the drop point', ln)
            if g:
                ev['drop'].append((t, float(g.group(1))))
            g = re.search(r'LIFT_PROOF: tip measured ([-+\d.]+) m from the contact target', ln)
            if g:
                ev['liftproof'].append((t, float(g.group(1))))
    return ev


# ------------------------------------------------------------------------------------------------------------ the bag
def read_bag(folder):
    cache = os.environ.get('JOINS_CACHE')
    if cache:
        fn = os.path.join(cache, folder.replace('/', '_') + '.pkl')
        if os.path.exists(fn):
            import pickle
            with open(fn, 'rb') as f:
                return pickle.load(f)
        arr = _read_bag(folder)
        import pickle
        os.makedirs(cache, exist_ok=True)
        with open(fn + '.tmp%d' % os.getpid(), 'wb') as f:
            pickle.dump(arr, f)
        os.replace(fn + '.tmp%d' % os.getpid(), fn)
        return arr
    return _read_bag(folder)


def _read_bag(folder):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    dbs = sorted(glob.glob(os.path.join(folder, '*.db3')))
    out = {k: [] for k in TOPICS}
    t_first = math.inf
    t_last = -math.inf
    for db in dbs:
        c = sqlite3.connect(db)
        tops = {n: (i, ty) for i, n, ty in c.execute('select id,name,type from topics')}
        a, b = c.execute('select min(timestamp),max(timestamp) from messages').fetchone()
        if a is not None:
            t_first, t_last = min(t_first, a * 1e-9), max(t_last, b * 1e-9)
        for key, name in TOPICS.items():
            if name not in tops:
                continue
            i, ty = tops[name]
            cls = get_message(ty)
            for ts, d in c.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
                m = deserialize_message(d, cls)
                t = ts * 1e-9
                if key == 'phase':
                    out[key].append((t, m.data))
                elif key == 'ref':
                    if m.points and m.points[0].transforms:
                        p = m.points[0].transforms[0].translation
                        out[key].append((t, p.x, p.y, p.z))
                else:
                    p, q = m.pose.position, m.pose.orientation
                    out[key].append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        c.close()
    arr = {}
    for k, v in out.items():
        if k == 'phase':
            arr[k] = v
        else:
            arr[k] = np.array(sorted(v)) if v else np.zeros((0, 8 if k != 'ref' else 4))
    arr['t0'] = t_first
    arr['t1'] = t_last
    return arr


def at(arr, t):
    """The latest sample at or before t (sample and hold), or None."""
    if len(arr) == 0:
        return None
    k = np.searchsorted(arr[:, 0], t, side='right') - 1
    if k < 0 or t - arr[k, 0] > 0.1:
        return None
    return arr[k]


def ring_frame(ring, ts, pts):
    """Points pts (N x 3, world) at times ts in the ring frame (R^T (p - ring)), with the ring sample held."""
    k = np.clip(np.searchsorted(ring[:, 0], ts, side='right') - 1, 0, len(ring) - 1)
    rr = ring[k]
    rel = Rot.from_quat(rr[:, 4:8]).inv().apply(pts - rr[:, 1:4])
    return rel, ring[k, 0]


# ----------------------------------------------------------------------------------------------------- the descents
def descents_from_phases(phases, start_phase, hold_phases):
    """[(t0, t1, from_phase, end_phase, end_reason)] for each entry into start_phase."""
    out = []
    for k, (t, a, b, why) in enumerate(phases):
        if b != start_phase:
            continue
        end = None
        for t2, a2, b2, why2 in phases[k + 1:]:
            if a2 in (start_phase,) + hold_phases and b2 not in (start_phase,) + hold_phases:
                end = (t2, b2, why2)
                break
        out.append((t, end[0] if end else None, a, end[1] if end else 'END', end[2] if end else 'recording ended'))
    return out


def _yaw(q):
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def plate_level(ring, ts, pts, weld):
    """Points pts (N x 3, world) at times ts against the plate point: (horizontal offset along the plate's radius
    (+ outward) and anticlockwise seen from above, N x 2: the plate's radial direction projected on the horizontal
    plane, so the body 1 m up is not thrown sideways by the ring's tilt; height along the ring normal above the weld
    point (N); offset in the full ring frame (N x 3), as our contact check reads it)."""
    k = np.clip(np.searchsorted(ring[:, 0], ts, side='right') - 1, 0, len(ring) - 1)
    rr = ring[k]
    rot = Rot.from_quat(rr[:, 4:8])
    weld = np.array(weld)
    plate = rr[:, 1:4] + rot.apply(weld)
    d = pts - plate
    u = rot.apply(np.array([weld[0], weld[1], 0.0]))[:, :2]
    u = u / np.linalg.norm(u, axis=1, keepdims=True)
    rad = u[:, 0] * d[:, 0] + u[:, 1] * d[:, 1]
    tan = -u[:, 1] * d[:, 0] + u[:, 0] * d[:, 1]
    full = rot.inv().apply(d)
    return np.stack([rad, tan], 1), full[:, 2], full


def plate_metrics(B, t0, t1, weld, contact_t=None):
    """Offsets of one plate descent; dict of columns. Horizontal offsets in the level frame (ring yaw only, so the
    body 1 m up is not thrown sideways by the ring's tilt); the height and the contact-comparable xy in the ring frame."""
    tip, body, ring, ref = B['tip'], B['body'], B['ring'], B['ref']
    w = (tip[:, 0] >= t0) & (tip[:, 0] <= t1) if len(tip) else np.zeros(0, bool)
    out = {'n_tip': int(w.sum())}
    if w.sum() < 5 or len(ring) < 5:
        out['note'] = 'no tip or ring samples in the descent'
        return out
    T = tip[w]
    lvl, normal, full = plate_level(ring, T[:, 0], T[:, 1:4], weld)
    hit = np.flatnonzero(normal <= 0.0)
    if len(hit):
        k = int(hit[0])
        out['eval'] = 'plate height'
    elif normal.min() <= PLATE_BAND:
        k = int(np.argmin(normal))
        out['eval'] = 'plate height'          # settled on the plate a few mm above the weld point (a tilted magnet)
    else:
        k = int(np.argmin(normal))
        out['eval'] = 'lowest point'
    # the lowest tip sample of the descent (where a magnet that slid off the plate ended up), for comparison
    kl = int(np.argmin(normal))
    out['low_tip_rad_cm'] = 100 * float(lvl[kl, 0])
    out['low_tip_tan_cm'] = 100 * float(lvl[kl, 1])
    out['low_tip_xy_cm'] = 100 * float(np.hypot(*lvl[kl]))
    bl_ = at(body, float(T[kl, 0]))
    if bl_ is not None:
        q, _, _ = plate_level(ring, np.array([T[kl, 0]]), bl_[None, 1:4], weld)
        out['low_body_rad_cm'] = 100 * float(q[0, 0])
        out['low_swing_rad_cm'] = out['low_tip_rad_cm'] - out['low_body_rad_cm']
    te = float(T[k, 0])
    out['t_eval'] = te
    out['eval_after_start_s'] = te - t0
    out['tip_normal_cm'] = 100 * normal[k]
    out['tip_min_normal_cm'] = 100 * float(normal.min())
    out['tip_rad_cm'] = 100 * float(lvl[k, 0])
    out['tip_tan_cm'] = 100 * float(lvl[k, 1])
    out['tip_xy_cm'] = 100 * float(np.hypot(*lvl[k]))
    out['tip_xy_ring_cm'] = 100 * float(np.hypot(*full[k, :2]))
    band = normal <= max(PLATE_BAND, float(normal.min()) + 0.02)
    out['tip_best_xy_cm'] = 100 * float(np.min(np.hypot(lvl[band, 0], lvl[band, 1])))
    out['tip_xy_mean_band_cm'] = 100 * float(np.mean(np.hypot(lvl[band, 0], lvl[band, 1])))
    if contact_t is not None:
        j = int(np.clip(np.searchsorted(T[:, 0], contact_t, side='right') - 1, 0, len(T) - 1))
        out['contact_tip_xy_ring_mm'] = 1000 * float(np.hypot(*full[j, :2]))
        out['contact_tip_normal_mm'] = 1000 * float(normal[j])
        out['contact_tip_rad_cm'] = 100 * float(lvl[j, 0])
        out['contact_tip_tan_cm'] = 100 * float(lvl[j, 1])
        bc = at(body, float(T[j, 0]))
        if bc is not None:
            bl, _, _ = plate_level(ring, np.array([T[j, 0]]), bc[None, 1:4], weld)
            out['contact_body_rad_cm'] = 100 * float(bl[0, 0])
            out['contact_body_tan_cm'] = 100 * float(bl[0, 1])
            out['contact_swing_rad_cm'] = out['contact_tip_rad_cm'] - out['contact_body_rad_cm']
            out['contact_swing_tan_cm'] = out['contact_tip_tan_cm'] - out['contact_body_tan_cm']
    b = at(body, te)
    if b is not None:
        bl, bn, _ = plate_level(ring, np.array([te]), b[None, 1:4], weld)
        out['body_rad_cm'] = 100 * float(bl[0, 0])
        out['body_tan_cm'] = 100 * float(bl[0, 1])
        out['body_height_m'] = float(bn[0])
        out['swing_rad_cm'] = out['tip_rad_cm'] - out['body_rad_cm']
        out['swing_tan_cm'] = out['tip_tan_cm'] - out['body_tan_cm']
        # QUAD4's motion relative to the plate over the last 1 s
        wb = (body[:, 0] >= te - 1.0) & (body[:, 0] <= te)
        if wb.sum() > 10:
            Bw = body[wb]
            xy, _, _ = plate_level(ring, Bw[:, 0], Bw[:, 1:4], weld)
            ker = np.ones(5) / 5
            xs = np.stack([np.convolve(xy[:, j], ker, mode='valid') for j in (0, 1)], 1)
            dt = Bw[-1, 0] - Bw[0, 0]
            out['body_speed_1s'] = float(np.sum(np.linalg.norm(np.diff(xs, axis=0), axis=1)) / max(dt, 1e-3))
            out['body_move_rad_1s_cm'] = 100 * float(xy[-1, 0] - xy[0, 0])
            out['body_move_tan_1s_cm'] = 100 * float(xy[-1, 1] - xy[0, 1])
        # his reference against the plate, and QUAD4 against his reference
        r = at(ref, te) if len(ref) else None
        if r is not None:
            rl, _, _ = plate_level(ring, np.array([te]), r[None, 1:4], weld)
            out['ref_rad_cm'] = 100 * float(rl[0, 0])
            out['ref_tan_cm'] = 100 * float(rl[0, 1])
            out['body_minus_ref_rad_cm'] = out['body_rad_cm'] - out['ref_rad_cm']
            out['body_minus_ref_tan_cm'] = out['body_tan_cm'] - out['ref_tan_cm']
    # tip speed relative to the plate at the reading (0.2 s window)
    ww = (T[:, 0] >= te - 0.1) & (T[:, 0] <= te + 0.1)
    if ww.sum() > 3:
        d = lvl[ww]
        out['tip_rel_speed'] = float(np.linalg.norm(d[-1] - d[0]) / max(T[ww][-1, 0] - T[ww][0, 0], 1e-3))
    # ring tilt and plate speed at the reading
    rk = at(ring, te)
    if rk is not None:
        z = Rot.from_quat(rk[4:8]).apply([0, 0, 1])
        out['ring_tilt_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, z[2]))))
        wr = (ring[:, 0] >= te - 0.25) & (ring[:, 0] <= te + 0.25)
        if wr.sum() > 5:
            R = ring[wr]
            P = R[:, 1:4] + Rot.from_quat(R[:, 4:8]).apply(np.array(weld))
            out['plate_speed'] = float(np.linalg.norm(P[-1, :2] - P[0, :2]) / max(R[-1, 0] - R[0, 0], 1e-3))
    return out


def object_metrics(B, t0, t1, contact_h, t_next=math.inf):
    tip, body, obj = B['tip'], B['body'], B['obj']
    w = (tip[:, 0] >= t0) & (tip[:, 0] <= t1) if len(tip) else np.zeros(0, bool)
    out = {'n_tip': int(w.sum())}
    if len(obj) < 5:
        out['note'] = 'no pickup-object pose in the bag'
        return out
    if w.sum() < 5:
        out['note'] = 'no tip samples in the descent'
        return out
    T = tip[w]
    k_o = np.clip(np.searchsorted(obj[:, 0], T[:, 0], side='right') - 1, 0, len(obj) - 1)
    O = obj[k_o]
    gap = T[:, 3] - O[:, 3] - contact_h
    d = T[:, 1:3] - O[:, 1:3]
    hit = np.flatnonzero(gap <= 0.0)
    if len(hit):
        k = int(hit[0])
        out['eval'] = 'target height'
    elif gap.min() <= PLATE_BAND:
        k = int(np.argmin(gap))
        out['eval'] = 'target height'
    else:
        k = int(np.argmin(gap))
        out['eval'] = 'lowest point'
    te = float(T[k, 0])
    out['t_eval'] = te
    out['eval_after_start_s'] = te - t0
    out['tip_normal_cm'] = 100 * float(gap[k])
    out['tip_min_normal_cm'] = 100 * float(gap.min())
    out['tip_x_cm'] = 100 * float(d[k, 0])
    out['tip_y_cm'] = 100 * float(d[k, 1])
    out['tip_xy_cm'] = 100 * float(np.hypot(*d[k]))
    band = gap <= max(PLATE_BAND, float(gap.min()) + 0.02)
    out['tip_best_xy_cm'] = 100 * float(np.min(np.hypot(d[band, 0], d[band, 1])))
    out['tip_xy_mean_band_cm'] = 100 * float(np.mean(np.hypot(d[band, 0], d[band, 1])))
    # did the object go up with the magnet (in the 20 s after the descent, before the next one)?
    wo = (obj[:, 0] >= t1) & (obj[:, 0] <= min(t1 + 20.0, t_next))
    if wo.sum() > 5:
        out['obj_rise_m'] = float(obj[wo, 3].max() - O[0, 3])
    b = at(body, te)
    if b is not None:
        o = at(obj, te)
        out['body_x_cm'] = 100 * float(b[1] - o[1])
        out['body_y_cm'] = 100 * float(b[2] - o[2])
        out['body_height_m'] = float(b[3] - o[3])
        out['swing_x_cm'] = out['tip_x_cm'] - out['body_x_cm']
        out['swing_y_cm'] = out['tip_y_cm'] - out['body_y_cm']
        wb = (body[:, 0] >= te - 1.0) & (body[:, 0] <= te)
        if wb.sum() > 10:
            Bw = body[wb]
            ker = np.ones(5) / 5
            xs = np.stack([np.convolve(Bw[:, j], ker, mode='valid') for j in (1, 2)], 1)
            out['body_speed_1s'] = float(np.sum(np.linalg.norm(np.diff(xs, axis=0), axis=1)) /
                                         max(Bw[-1, 0] - Bw[0, 0], 1e-3))
    return out


def drop_metrics(B, t_drop):
    """The tip over the ring at the release, and where the object first hit the net (relative to the ring centre)."""
    tip, ring, obj = B['tip'], B['ring'], B['obj']
    out = {'t_eval': t_drop}
    r = at(ring, t_drop)
    tp = at(tip, t_drop)
    if r is not None and tp is not None:
        out['tip_x_cm'] = 100 * float(tp[1] - r[1])
        out['tip_y_cm'] = 100 * float(tp[2] - r[2])
        out['tip_xy_cm'] = 100 * float(np.hypot(tp[1] - r[1], tp[2] - r[2]))
        out['tip_normal_cm'] = 100 * float(tp[3] - r[3])
    if len(obj) < 5 or len(ring) < 5:
        out['note'] = 'no object or ring pose'
        return out
    w = (obj[:, 0] >= t_drop) & (obj[:, 0] <= t_drop + 2.0)
    O = obj[w]
    if len(O) < 10:
        out['note'] = 'object pose missing after the release'
        return out
    k_r = np.clip(np.searchsorted(ring[:, 0], O[:, 0], side='right') - 1, 0, len(ring) - 1)
    R = ring[k_r]
    rel = Rot.from_quat(R[:, 4:8]).inv().apply(O[:, 1:4] - R[:, 1:4])
    rz = np.convolve(rel[:, 2], np.ones(3) / 3, mode='same')
    vz = np.gradient(rz, O[:, 0])
    # first impact: the object's fall relative to the ring stops (back above -0.3 m/s after falling faster than -1 m/s)
    fell = np.flatnonzero(vz[1:-1] < -1.0) + 1
    out['obj_fall_m'] = None
    if len(fell):
        after = np.flatnonzero((np.arange(len(vz)) > fell[0]) & (vz > -0.3))
        if len(after):
            k = int(after[0])
            out['impact_after_s'] = float(O[k, 0] - t_drop)
            out['impact_xy_cm'] = 100 * float(np.hypot(rel[k, 0], rel[k, 1]))
            out['impact_height_cm'] = 100 * float(rel[k, 2])
            out['impact_speed'] = float(-vz[max(1, k - 5):k].min()) if k > 1 else math.nan
            out['obj_fall_m'] = float(rel[0, 2] - rel[k, 2])
    # where the object settled (the last 0.3 s of the 2 s window), relative to the ring centre
    s = O[:, 0] >= O[-1, 0] - 0.3
    out['settled_xy_cm'] = 100 * float(np.mean(np.hypot(rel[s, 0], rel[s, 1])))
    out['settled_height_cm'] = 100 * float(np.mean(rel[s, 2]))
    return out


def start_weld(B, weld):
    """The tip in the ring frame before anything moves (the first 2 s with the ring on the floor), minus the start
    plate's weld point."""
    tip, ring = B['tip'], B['ring']
    if len(tip) < 20 or len(ring) < 20:
        return None
    t0 = max(tip[0, 0], ring[0, 0])
    w = (tip[:, 0] >= t0) & (tip[:, 0] <= t0 + 2.0) & (tip[:, 3] < 0.3)
    if w.sum() < 10:
        return None
    rel, _ = ring_frame(ring, tip[w, 0], tip[w, 1:4])
    m = np.median(rel, axis=0)
    return {'weld_meas': m, 'angle_cw_deg': (-math.degrees(math.atan2(m[1], m[0]))) % 360.0, 'd_xy_mm': 1000 * float(np.hypot(m[0] - weld[0], m[1] - weld[1])),
            'd_z_mm': 1000 * float(m[2] - weld[2]), 'r_mm': 1000 * float(np.hypot(m[0], m[1])),
            'ring_rest_z_mm': 1000 * float(np.median(ring[(ring[:, 0] <= t0 + 2.0), 3]))}


# ------------------------------------------------------------------------------------------------------------- main
def formation(args):
    az = args.get('attach_azimuths_deg', '')
    plates = sorted(int(round(float(a) / 30.0)) % 12 for a in az.split(',') if a)
    if plates == [3, 7, 11]:
        return 'triangle', '3/11/7'
    if plates == [2, 5, 11]:
        return 'square', '11/2/5'
    return 'other', '/'.join(map(str, plates))


def kind_of(args, name):
    mode = args.get('mission_mode', 'floor_join')
    welded = args.get('partner_attached', 'false') == 'true'
    if mode.startswith('pickup'):
        return 'pickup', welded
    return ('re-attach' if welded else 'attach from the mat'), welded


def main():
    regs = registry()
    flights, rows = [], []
    only = sys.argv[1:]
    for L in launches():
        if L['flew'] != 'yes' or (only and L['name'] not in only and f"{L['day']}/{L['name']}" not in only):
            continue
        day, name, args = L['day'], L['name'], L['args']
        rid = f'RIG-{day[5:7]}{day[8:10]}-{name}'
        corr = rid + 'c' if rid + 'c' in regs else ''
        log = parse_log(os.path.join(RIG, day, name + '.log'))
        bagdir = os.path.join(RIG, day, name)
        form, carriers = formation(args)
        kind, welded = kind_of(args, name)
        motion = args.get('load_traj', 'hover')
        plate = plate_of(args)
        ct = log['contact_target'] or (float(args.get('attach_x_offset', 0)), float(args.get('attach_y_offset', 0.225)),
                                      float(args.get('weld_point_z', 0.0365)))
        typed = ct
        note = []
        if day == '2026-11-05' and abs(ct[2] - WELD_5NOV[2]) > 1e-4:
            xy = turned(WELD_5NOV[:2], plate)
            ct = (xy[0], xy[1], WELD_5NOV[2])
            note.append(f'typed 4 Nov QUAD4 values (W {typed[2]}, ({typed[0]}, {typed[1]})); scored against 5 Nov W/X/Y')
        geom = log['geometry']
        cfg_ring = (f'{geom[1]:.2f} kg config (r {geom[0]:.3f})' if geom else '?')
        print(f'{rid}: reading the bag ...', flush=True)
        try:
            B = read_bag(bagdir)
        except Exception as ex:  # noqa: BLE001
            print(f'  unreadable bag: {ex}')
            B = None
        start_plate = None
        sw = None
        if B is not None and welded:
            start_plate = int(round(log['start_plate_deg'] / 30.0)) % 12 if log.get('start_plate_deg') is not None else 9
            w9 = WELD_5NOV if day == '2026-11-05' else (float(args.get('attach_x_offset', 0.0039)),
                                                        float(args.get('attach_y_offset', 0.2271)),
                                                        float(args.get('weld_point_z', 0.0365)))
            sxy = turned(w9[:2], start_plate)
            sw = start_weld(B, (sxy[0], sxy[1], w9[2]))
        rest_z = None
        if B is not None and len(B['ring']):
            rest_z = 1000 * float(np.median(B['ring'][B['ring'][:, 0] <= B['ring'][0, 0] + 2.0, 3]))
        reg = regs.get(rid, [])
        cfg_txt = reg[2] if len(reg) > 2 else ''
        phys = ('M2A 0.86 kg' if 'M2A' in cfg_txt else '1.15 kg' if '1.15 kg ring' in cfg_txt
                else 'M2A 0.86 kg' if 'M2A' in ' '.join(reg) else '?')
        fl = {'day': day, 'flight': name, 'registry_id': rid, 'correction_row': corr, 'kind': kind,
              'formation': form, 'carriers': carriers, 'motion': motion, 'plate': plate if kind != 'pickup' or welded else '',
              'config_ring': cfg_ring, 'ring_rest_z_mm': f'{rest_z:.1f}' if rest_z is not None else '',
              'ring_registry': phys, 'target_xy': f'{ct[0]:.4f},{ct[1]:.4f}', 'target_z': f'{ct[2]:.4f}',
              'start_plate_log': start_plate if start_plate is not None else '',
              'start_weld_angle_cw': f'{sw["angle_cw_deg"]:.1f}' if sw else '',
              'start_weld_dxy_mm': f'{sw["d_xy_mm"]:.1f}' if sw else '', 'start_weld_dz_mm': f'{sw["d_z_mm"]:.1f}' if sw else '',
              'start_weld_r_mm': f'{sw["r_mm"]:.1f}' if sw else '',
              'bag_s': f'{B["t1"] - B["t0"]:.1f}' if B is not None else '',
              'n_tip': len(B['tip']) if B is not None else 0, 'n_body': len(B['body']) if B is not None else 0,
              'n_ring': len(B['ring']) if B is not None else 0, 'n_obj': len(B['obj']) if B is not None else 0,
              'his_takeoff': '', 'release': '', 'n_descents': 0, 'note': '; '.join(note)}
        if B is None:
            flights.append(fl)
            continue
        t_bag = B['t0']
        t_to = log['his_takeoff'][0] if log['his_takeoff'] else None
        t_rel = log['release'][0] if log['release'] else None
        fl['his_takeoff'] = f'{t_to - t_bag:.1f}' if t_to else ''
        fl['release'] = f'{t_rel - t_bag:.1f}' if t_rel else ''
        # the ring descents
        ring_d = descents_from_phases(log['phases'], 'DESCEND_TO_ATTACHMENT', ('ATTACH_READY',))
        pick_d = descents_from_phases(log['phases'], 'DESCEND_TO_PICKUP', ('MAGNET_ATTACH_WAIT',))
        all_d = [('ring', d) for d in ring_d] + [('pickup', d) for d in pick_d]
        all_d.sort(key=lambda x: x[1][0])
        fl['n_descents'] = len(all_d)
        n_ring = 0
        n_pick = 0
        for what, (t0, t1, frm, endp, why) in all_d:
            t1 = t1 if t1 is not None else B['t1']
            nxt = [d[1][0] for d in all_d if d[1][0] > t0]
            t_next = min(nxt) if nxt else math.inf
            row = {'day': day, 'flight': name, 'registry_id': rid + (f' (+{corr})' if corr else ''),
                   'formation': form, 'carriers': carriers, 'ring_motion': motion,
                   'descent_start_s': round(t0 - t_bag, 2), 'descent_end_s': round(t1 - t_bag, 2),
                   'descent_from': frm, 'end_phase': endp, 'end_reason': why}
            if what == 'ring':
                n_ring += 1
                row['kind'] = 'attach from the mat' if not welded else 're-attach'
                row['target'] = f'plate {plate}'
                row['n'] = n_ring
                # contact and proof
                con = [c for c in log['contacts'] if t0 - 0.2 <= c[0] <= t1 + 0.3]
                outcome = None
                if endp == 'LIFT_PROOF' or con:
                    c = con[0] if con else None
                    if c:
                        row['contact_log_xy_mm'] = c[2]
                        row['contact_log_normal_mm'] = c[3]
                        row['contact_try'] = c[1]
                    pf = [p for p in log['proof_failed'] if t1 - 0.1 <= p[0] < t_next + 0.5]
                    pv = [p for p in log['proven'] if t1 - 0.1 <= p[0] < t_next + 0.5]
                    if pv:
                        outcome = 'caught+proven'
                        row['proof'] = f'proven, drift {pv[0][2]:.1f} mm'
                        row['t_proven_s'] = round(pv[0][0] - t_bag, 2)
                        t_ref = t_rel if welded else t_to
                        if t_ref:
                            row['time_to_proven_s'] = round(pv[0][0] - t_ref, 1)
                            r4 = [r for r in log['resize4'] if r > pv[0][0] - 0.5]
                            if r4:
                                row['time_to_resize4_s'] = round(r4[0] - t_ref, 1)
                        row['time_from'] = 'release' if welded else 'his TAKEOFF'
                    elif pf:
                        outcome = 'contact not proven'
                        row['proof'] = f'failed ({pf[0][2]}): drift {pf[0][3]:.1f} mm, plate rise {pf[0][4]:.1f} mm'
                    else:
                        outcome = 'contact not proven'
                        row['proof'] = 'no proof result logged'
                elif endp == 'APPROACH_ABOVE_TARGET' and 'sunk below' in why:
                    outcome = 'miss'
                    row['miss_kind'] = 'sank off the plate edge'
                elif endp == 'APPROACH_ABOVE_TARGET' and 'quiet miss' in why:
                    outcome = 'miss'
                    row['miss_kind'] = 'quiet miss (no progress)'
                elif endp == 'APPROACH_ABOVE_TARGET' and 'moving or far' in why:
                    outcome = 'miss'
                    row['miss_kind'] = 'moving or far off'
                elif endp == 'APPROACH_ABOVE_TARGET':
                    outcome = 'aborted'
                    row['miss_kind'] = 'alignment lost in the descent'
                elif endp == 'TAKEN_BACK':
                    outcome = 'aborted'
                    row['miss_kind'] = 'take-back (LAND / give-up)'
                else:
                    outcome = 'aborted'
                    row['miss_kind'] = why
                row['outcome'] = outcome
                row['holds_in_descent'] = sum(1 for h in log['held'] if t0 <= h <= t1)
                m = plate_metrics(B, t0, t1, ct, con[0][0] if con else None)
            else:
                n_pick += 1
                row['kind'] = 'pickup'
                row['target'] = 'object'
                row['n'] = n_pick
                ch = float(args.get('pickup_contact_height', 0.018))
                att = [p for p in log['pick_attached'] if t0 <= p[0] <= t1 + 0.3]
                fail = [p for p in log['pick_failed'] if t0 <= p[0] < min(t_next, t1 + 5.0)]
                lifted = [p for p in log['lifted'] if t0 <= p[0] < t_next]
                if att:
                    row['contact_log_xy_mm'] = 1000 * att[0][1]
                if lifted:
                    row['outcome'] = 'caught+proven'
                    row['proof'] = f'object lifted {lifted[0][1]:.2f} m'
                    if t_to:
                        row['time_to_proven_s'] = round(lifted[0][0] - t_to, 1)
                        row['time_from'] = 'his TAKEOFF'
                elif att:
                    row['outcome'] = 'contact not proven'
                    row['proof'] = (f'attached, then: {fail[0][2]}' if fail else 'attached, no lift logged')
                elif fail:
                    row['outcome'] = 'contact not proven' if 'object attached' in fail[0][2] else 'miss'
                    row['proof'] = fail[0][2]
                    if not math.isnan(fail[0][3]):
                        row['contact_log_xy_mm'] = 1000 * fail[0][3]
                elif endp == 'TAKEN_BACK':
                    row['outcome'] = 'aborted'
                    row['miss_kind'] = 'take-back (LAND / give-up)'
                else:
                    row['outcome'] = 'miss'
                    row['miss_kind'] = why
                m = object_metrics(B, t0, t1, ch, t_next)
                rise = m.get('obj_rise_m', math.nan)
                if row['outcome'] in ('contact not proven', 'miss') and rise > 0.2:
                    row['outcome'] = 'caught, not proven'
                    row['proof'] = (row.get('proof') or '') + f'; the object rose {rise:.2f} m with the magnet'
            for k, v in m.items():
                if k == 't_eval':
                    row['t_eval_s'] = round(v - t_bag, 2)
                else:
                    row[k] = v
            rows.append(row)
        # drops into the net
        for t_d, xy_log in log['drop']:
            row = {'day': day, 'flight': name, 'registry_id': rid + (f' (+{corr})' if corr else ''),
                   'formation': form, 'carriers': carriers, 'ring_motion': motion, 'kind': 'drop into the net',
                   'target': 'ring centre (net)', 'n': 1, 'descent_start_s': round(t_d - t_bag, 2),
                   'contact_log_xy_mm': 1000 * xy_log}
            m = drop_metrics(B, t_d)
            for k, v in m.items():
                if k == 't_eval':
                    row['t_eval_s'] = round(v - t_bag, 2)
                else:
                    row[k] = v
            hit = m.get('impact_xy_cm', math.nan)
            row['outcome'] = ('hit the net' if hit < 22.0 else 'missed the net' if hit >= 22.0 else 'not measured')
            later = [t for t in log['payload_dropped'] if t_d < t <= t_d + 3.0]
            if later:
                row['note'] = f'the ring was dropped {later[0] - t_d:.1f} s after the release'
            rows.append(row)
        fl['n_descents'] = len(all_d)
        flights.append(fl)

    cols = ['day', 'flight', 'registry_id', 'kind', 'formation', 'carriers', 'ring_motion', 'target', 'n',
            'descent_from', 'descent_start_s', 'descent_end_s', 'end_phase', 'end_reason', 'outcome', 'miss_kind', 'proof',
            'eval', 't_eval_s', 'eval_after_start_s', 'tip_normal_cm', 'tip_min_normal_cm',
            'tip_rad_cm', 'tip_tan_cm', 'tip_x_cm', 'tip_y_cm', 'tip_xy_cm', 'tip_xy_ring_cm', 'tip_best_xy_cm',
            'tip_xy_mean_band_cm',
            'low_tip_rad_cm', 'low_tip_tan_cm', 'low_tip_xy_cm', 'low_body_rad_cm', 'low_swing_rad_cm',
            'body_rad_cm', 'body_tan_cm', 'body_x_cm', 'body_y_cm', 'body_height_m',
            'swing_rad_cm', 'swing_tan_cm', 'swing_x_cm', 'swing_y_cm',
            'body_speed_1s', 'body_move_rad_1s_cm', 'body_move_tan_1s_cm', 'tip_rel_speed',
            'ref_rad_cm', 'ref_tan_cm', 'body_minus_ref_rad_cm', 'body_minus_ref_tan_cm', 'ring_tilt_deg', 'plate_speed',
            'contact_log_xy_mm', 'contact_log_normal_mm', 'contact_try', 'contact_tip_xy_ring_mm', 'contact_tip_normal_mm',
            'contact_tip_rad_cm', 'contact_tip_tan_cm', 'contact_body_rad_cm', 'contact_body_tan_cm', 'contact_swing_rad_cm',
            'contact_swing_tan_cm', 'holds_in_descent',
            'time_from', 'time_to_proven_s', 'time_to_resize4_s', 't_proven_s',
            'obj_rise_m', 'impact_after_s', 'impact_xy_cm', 'impact_height_cm', 'impact_speed', 'obj_fall_m', 'settled_xy_cm',
            'settled_height_cm', 'n_tip', 'note']

    def fmt(v):
        if isinstance(v, float):
            return '' if not math.isfinite(v) else f'{v:.3f}' if abs(v) < 10 else f'{v:.2f}'
        return '' if v is None else v
    with open(os.path.join(OUT, 'joins.csv'), 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore', lineterminator='\n')
        wr.writeheader()
        for r in rows:
            wr.writerow({k: fmt(r.get(k)) for k in cols})
    fcols = list(flights[0].keys())
    with open(os.path.join(OUT, 'flights.csv'), 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=fcols, lineterminator='\n')
        wr.writeheader()
        for r in flights:
            wr.writerow(r)
    print(f'wrote {len(rows)} rows to joins.csv, {len(flights)} flights to flights.csv')
    return 0


if __name__ == '__main__':
    sys.exit(main())
