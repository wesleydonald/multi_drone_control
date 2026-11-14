"""Shared pieces for the 13 Nov C++ rig analysis: log parsing, the 100 Hz pose grid, the cable geometry."""
import os
import re
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
RIG = os.path.join(ROOT, 'results', 'rig', '2026-11-13')
FLIGHTS = ['fm9h8_3', 'fm9h8_4', 'fm9h8_5', 'fm9hc', 'fm9hc_5', 'fm9hc_8', 'fm9h8_7', 'fm9h8_10', 'fm9h8_13',
           'fm9h8_14', 'fm9hc_9', 'fm9hc_10', 'fm9h_2', 'fm9hc_11']
FROZEN = ['fm9h8_13', 'fm9hc_10', 'fm9h_2', 'fm9hc_11']

R_PLATE, Z_PLATE = 0.225, 0.018   # M2A ring plates: radius and height over the ring marker (ring body frame)
PIVOT = 0.04                      # every cable leaves its quad 0.04 m below the body centre
OBJ_HALF = 0.0525                 # the object's half-size (0.105 m), taken off its marker distance
OBJ_ON_TIP = 0.15                 # object marker within this of the tip = still on the magnet (mocap jumps dropped)
GRID_DT = 0.01
GAP_MAX = 0.10                    # a pose older than this on the grid is a dropout (NaN)

ARGS = ('max_swing_angle_rad', 'spline_time_factor', 'cpp_splice', 'cpp_max_z', 'drop_height', 'max_body_z',
        'load_traj', 'traj_speed', 'fig8_size')


def stamp_local(t):
    return time.strftime('%H:%M:%S', time.localtime(t))


def parse_log(name):
    """Phases, his C++ authority events, fallbacks, our LAND, position limits and the typed args from <name>.log."""
    out = {'phases': [], 'fallbacks': [], 'land_cmd': [], 'pos_limit': [], 'stale_ref': [], 'args': {},
           'cpp_dir': None, 'bridge_done': None, 'authority_switch': None, 'climb_done': None, 'drop_reason': None,
           'disarm': [], 'backend_disabled': []}
    rx_ph = re.compile(r'\[(\d+\.\d+)\] \[online_join_planner\]: Planner phase: (\S+) -> (\S+)\. Reason: (.*)')
    rx_fb = re.compile(r'\[(\d+\.\d+)\] \[dynamic_planner_transfer_backend\]: C1F\.4 execution certificate invalid '
                       r'\((.*?)\)\. Installed (\S+) fallback')
    rx_t = re.compile(r'\[(\d+\.\d+)\]')
    for line in open(os.path.join(RIG, name + '.log'), errors='replace'):
        if 'C1F.6 C++ log directory:' in line and out['cpp_dir'] is None:
            out['cpp_dir'] = line.split('C1F.6 C++ log directory:')[1].strip()
        if ':=' in line and len(out['args']) < len(ARGS):
            for k in ARGS:
                m = re.search(r'\b' + k + r':=([^ \]]+)', line)
                if m and k not in out['args']:
                    out['args'][k] = m.group(1)
        m = rx_ph.search(line)
        if m:
            out['phases'].append((float(m[1]), m[2], m[3], m[4].strip()))
            if m[3] == 'DROP_OBJECT':
                out['drop_reason'] = m[4].strip()
            continue
        m = rx_fb.search(line)
        if m:
            out['fallbacks'].append((float(m[1]), m[3], m[2].split(' ')[0]))
            continue
        t = rx_t.search(line)
        if not t:
            continue
        ts = float(t[1])
        if '[fleet_manager]: Fleet command received: \'LAND\'' in line:
            out['land_cmd'].append(ts)
        elif '[fleet_manager]: POSITION LIMIT' in line:
            out['pos_limit'].append((ts, line.split('POSITION LIMIT:')[1].strip()[:80]))
        elif '[fleet_manager]: DISARM: disarming all drones' in line:
            out['disarm'].append(ts)
        elif 'moving-frame bridge complete' in line and out['bridge_done'] is None:
            out['bridge_done'] = ts
        elif 'authority source switched stationary Python hold -> C++' in line and out['authority_switch'] is None:
            out['authority_switch'] = ts
        elif 'loaded climb complete' in line and out['climb_done'] is None:
            out['climb_done'] = ts
        elif '[dynamic_planner_transfer_backend]: Backend disabled at ROS' in line:
            out['backend_disabled'].append((ts, line.split(': ', 2)[-1].split(';')[0].split(': ')[-1].strip()))
        elif 'C++ reference is stale/unavailable' in line:
            out['stale_ref'].append(ts)
    return out


def phase_t(log, to, after=None):
    """First time his planner entered phase `to` (after `after`), else None."""
    for t, a, b, why in log['phases']:
        if b == to and (after is None or t > after):
            return t
    return None


def next_phase(log, t0):
    for t, a, b, why in log['phases']:
        if t > t0 + 1e-6:
            return t, b, why
    return None, None, None


def load(name):
    return dict(np.load(os.path.join(HERE, 'cache', name + '.npz'), allow_pickle=False))


def grid(d, t0, t1, keys=('d0', 'd1', 'd2', 'd3', 'ring', 'obj', 'tip')):
    """All poses on one 100 Hz grid over [t0, t1]; samples further than GAP_MAX from a real one are NaN."""
    T = np.arange(t0, t1, GRID_DT)
    g = {'t': T}
    for k in keys:
        a = d[k]
        if len(a) < 2:
            g[k] = np.full((len(T), a.shape[1] - 1), np.nan)
            continue
        cols = [np.interp(T, a[:, 0], a[:, j]) for j in range(1, a.shape[1])]
        v = np.stack(cols, 1)
        i = np.clip(np.searchsorted(a[:, 0], T), 1, len(a) - 1)
        gap = np.minimum(np.abs(a[i, 0] - T), np.abs(a[i - 1, 0] - T))
        v[(gap > GAP_MAX) | (T < a[0, 0]) | (T > a[-1, 0])] = np.nan
        if v.shape[1] == 7:
            q = v[:, 3:7]
            v[:, 3:7] = q / np.linalg.norm(q, axis=1, keepdims=True)
        g[k] = v
    return g


def rot(q, v):
    """Rotate v (N,3) by unit quaternions q (N,4) given as (w, x, y, z)."""
    w, u = q[:, :1], q[:, 1:]
    t = 2.0 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def rot_inv(q, v):
    qc = q.copy()
    qc[:, 1:] *= -1.0
    return rot(qc, v)


def plate_pos(ring, k):
    """Plate k (clockwise from the ring's +x seen from above, 30 deg apart) in the world: ring pose (N,7)."""
    a = np.radians(-30.0 * k)
    off = np.broadcast_to(np.array([R_PLATE * np.cos(a), R_PLATE * np.sin(a), Z_PLATE]), (len(ring), 3))
    return ring[:, :3] + rot(ring[:, 3:7], off)


def seg_seg(p1, q1, p2, q2, eps=1e-12):
    """Closest distance between segments p1-q1 and p2-q2, row by row (copy of tools/m1_metrics.seg_seg)."""
    p1, q1, p2, q2 = np.broadcast_arrays(*(np.asarray(x, float) for x in (p1, q1, p2, q2)))
    dot = lambda a, b: np.einsum('...i,...i->...', a, b)
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f, c, b = dot(d1, d1), dot(d2, d2), dot(d2, r), dot(d1, r), dot(d1, d2)
    A, E = a < eps, e < eps
    a_, e_ = np.where(A, 1.0, a), np.where(E, 1.0, e)
    den = a * e - b * b
    ok = den > eps
    s = np.where(ok, np.clip((b * f - c * e) / np.where(ok, den, 1.0), 0.0, 1.0), 0.0)
    t = (b * s + f) / e_
    s = np.where(t < 0.0, np.clip(-c / a_, 0.0, 1.0), np.where(t > 1.0, np.clip((b - c) / a_, 0.0, 1.0), s))
    t = np.clip(t, 0.0, 1.0)
    s = np.where(E & ~A, np.clip(-c / a_, 0.0, 1.0), s)
    t = np.where(E, 0.0, t)
    s = np.where(A, 0.0, s)
    t = np.where(A & ~E, np.clip(f / e_, 0.0, 1.0), t)
    return np.linalg.norm(p1 + d1 * s[..., None] - p2 - d2 * t[..., None], axis=-1)


def cable_top(g, i):
    return g[f'd{i}'][:, :3] - np.array([0.0, 0.0, PIVOT])
