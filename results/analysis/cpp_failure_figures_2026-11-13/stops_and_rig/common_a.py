"""Shared loader for the moving-ring replay figures (Part A). Read-only on ../../cpp_realtime_2026-11-13.
Geometry as full/dist.py: QUAD4 hanging line = body -> body - 1.134 m (cable 1.034 + magnet + object, vertical, no swing);
carrier tether = carrier body - 0.04 m -> its plate (ring centre + 0.225 m toward the carrier, at ring height).
Replay time -> real nro poses: the offset is fitted on the ring track (same search as dist.py, around the 66 s bag offset)."""
import bisect
import csv
import math
import os

import numpy as np

SRC = '/home/wesley/multi_drone_control/results/analysis/cpp_realtime_2026-11-13'
LINE = 1.034 + 0.10
PLATE_R = 0.225
OUTER_R = 0.28   # moving_basket_collision_outer_diameter_m 0.56 / 2


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


class Run:
    def __init__(self, run, flight='nro', req=66.0):
        base = f'{SRC}/full/runs/{run}'
        self.name = run
        d = list(csv.DictReader(open(f'{base}/driver.csv')))
        self.b = list(csv.DictReader(open(f'{base}/backend.csv')))
        self.rp = list(csv.DictReader(open(f'{base}/replans.csv')))
        p = list(csv.DictReader(open(f'{SRC}/{flight}_poses.csv')))
        self.t0 = float(d[0]['t'])
        self.d = d
        self.t = np.array([float(r['t']) - self.t0 for r in d])
        for k in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'tx', 'ty', 'tz', 'ring_x', 'ring_y', 'ring_z', 'dxy', 'dz',
                  'min_body_m', 'min_cable_tether_m'):
            setattr(self, k, np.array([_f(r[k]) for r in d]))
        self.auth = np.array([r['auth'] == '1' for r in d])
        self.cap = np.array([r['captured'] == '1' for r in d])
        self.status = [r['status'] for r in d]
        self.speed = np.sqrt(self.vx ** 2 + self.vy ** 2 + self.vz ** 2)
        self.pt = np.array([float(r['t']) for r in p])
        self.p = p
        self.t_auth = self.t[self.auth][0] if self.auth.any() else None
        self.t_cap = self.t[self.cap][0] if self.cap.any() else None
        # fit replay -> flight offset on the ring track (as dist.py)
        best = None
        for off in [req - 4.0 + x * 0.02 for x in range(250)]:
            err = 0.0
            for i in range(0, len(d), 20):
                q = self._at(self.t[i] + off)
                err += math.hypot(self.ring_x[i] - float(q['r_x']), self.ring_y[i] - float(q['r_y']))
            if best is None or err < best[0]:
                best = (err, off)
        self.off = best[1]
        self.fit_err = best[0] / len(range(0, len(d), 20))
        # backend (1 Hz)
        self.bt = np.array([float(r['ros_time']) - self.t0 for r in self.b])
        self.commit_left = np.array([_f(r['commit_end_s']) - float(r['ros_time']) for r in self.b])
        self.fb_active = np.array([r['execution_fallback_active'] == '1' for r in self.b])
        self.fb_mode = [r['execution_fallback_mode'] for r in self.b]
        self.fb_reason = [r['execution_fallback_reason'] for r in self.b]
        self.splice_ms = 1000 * float([l for l in open(f'{base}/backend.yaml') if 'fixed_splice_lookahead_s' in l][0].split(':')[1])
        # replans
        self.rt = np.array([float(r['request_ros_time']) - self.t0 for r in self.rp])
        self.rstat = [r['status'] for r in self.rp]
        self.rinst = np.array([r['authority_installed'] == '1' for r in self.rp])
        self.rms = np.array([_f(r['replan_runtime_ms']) for r in self.rp])

    def _at(self, t):
        i = min(max(bisect.bisect_left(self.pt, t), 0), len(self.pt) - 1)
        return self.p[i]

    def scene(self, t):
        """Carriers, plates and ring at replay time t (driver clock, s from replay start)."""
        q = self._at(t + self.off)
        ring = np.array([float(q['r_x']), float(q['r_y']), float(q['r_z'])])
        rv = np.array([float(q['r_vx']), float(q['r_vy']), float(q['r_vz'])])
        cars = []
        for i in range(3):
            c = np.array([float(q[f'd{i}_x']), float(q[f'd{i}_y']), float(q[f'd{i}_z'])])
            h = c[:2] - ring[:2]
            plate = ring + np.array([*(PLATE_R * h / np.linalg.norm(h)), 0.0])
            cars.append((c, c - np.array([0, 0, 0.04]), plate))
        return ring, rv, cars

    def idx(self, t):
        return int(np.argmin(np.abs(self.t - t)))

    def still_runs(self, v=0.05):
        """[(start, end)] of QUAD4 still (< v m/s) after authority and before capture."""
        out, s = [], None
        for i, t in enumerate(self.t):
            ok = self.auth[i] and not self.cap[i] and self.speed[i] < v
            if ok and s is None:
                s = t
            if not ok and s is not None:
                out.append((s, self.t[i - 1]))
                s = None
        if s is not None:
            out.append((s, self.t[-1]))
        return out


def seg_seg(p1, q1, p2, q2):
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    c, b = d1 @ r, d1 @ d2
    den = a * e - b * b
    s = min(max((b * f - c * e) / den, 0.0), 1.0) if den > 1e-12 else 0.0
    t = (b * s + f) / e
    if t < 0.0:
        t, s = 0.0, min(max(-c / a, 0.0), 1.0)
    elif t > 1.0:
        t, s = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))


CATS = [  # (label, colour, predicate on status, installed)
    ('accepted and installed', '#1a9850', lambda s, i: s.startswith('ACCEPTED') and i),
    ('accepted, not installed', '#a6d96a', lambda s, i: s.startswith('ACCEPTED') and not i),
    ('rejected at final recheck\n(late / world changed / unsafe)', '#7b3294',
     lambda s, i: s.startswith(('LATE_', 'WORLD_CHANGED', 'LATEST_CANDIDATE_UNSAFE', 'INCUMBENT_PREFIX_UNSAFE'))),
    ('no plan found\n(LOCAL_PLAN_FAILED)', '#fdae61', lambda s, i: s.startswith('LOCAL_PLAN_FAILED')),
    ('no plan AND incumbent unsafe\n(INCUMBENT_REMAINING_UNSAFE_...)', '#d7191c',
     lambda s, i: s.startswith('INCUMBENT_REMAINING_UNSAFE')),
    ('target time not converged', '#999999', lambda s, i: s.startswith('TARGET_TIME')),
]


def cat_of(s, inst):
    for k, (_, _, f) in enumerate(CATS):
        if f(s, inst):
            return k
    return len(CATS) - 1
