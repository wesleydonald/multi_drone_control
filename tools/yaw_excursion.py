#!/usr/bin/env python3
"""
tools/yaw_excursion.py -- per-drone yaw excursion after TAKEOFF, from a run's tracker logs.

The metric of docs/experiments/2026-09-16_qref_hemisphere.md: for each drone,
max |yaw(t) - yaw(liftoff)| (unwrapped, deg) over the `window` seconds after liftoff
(first throttle above idle in that drone's own log), plus the yaw-stick saturation fraction in that window and whether the
run aborted. A healthy takeoff holds heading to a few degrees; the rig spin of
2026-09-16 was +180 deg in 1.4 s with the stick pinned.

    tools/yaw_excursion.py R0275 R0276              # harness run ids or paths
    tools/yaw_excursion.py results/logs/controller_quad_load --stamp 20260916_1640  # rig logs
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
from run_dir import resolve  # noqa: E402


def _yaw(w, x, y, z):
    return np.degrees(np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def _load(path):
    rows = list(csv.DictReader(open(path)))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}


def tracker_logs(run_path, stamp=None):
    pat = os.path.join(run_path, 'logs', 'controller_quad_load', 'planner_drone*') if stamp is None \
        else os.path.join(run_path, f'planner_drone*_{stamp}*')
    out = {}
    for d in sorted(glob.glob(pat)):
        i = int(os.path.basename(d).split('_')[1].replace('drone', ''))
        if os.path.exists(os.path.join(d, 'log.csv')):
            out[i] = os.path.join(d, 'log.csv')
    return out


def takeoff_time(run_path):
    ev = os.path.join(run_path, 'logs', 'events.csv')
    if os.path.exists(ev):
        for r in csv.DictReader(open(ev)):
            if r['event'].strip().upper() == 'TAKEOFF':
                return float(r['sim_time'])
    return None


def aborted(run_path):
    """FLEET_ABORT in the run's events.csv (the manifest's exit_reason says only
    'health check failed')."""
    ev = os.path.join(run_path, 'logs', 'events.csv')
    if not os.path.exists(ev):
        return None
    return any(r['event'].strip().upper() == 'FLEET_ABORT' for r in csv.DictReader(open(ev)))


def excursions(run_path, window=15.0, stamp=None):
    logs = tracker_logs(run_path, stamp)
    t_to = takeoff_time(run_path)
    res = {}
    for i, path in logs.items():
        d = _load(path)
        t = d['sim_time']
        # Anchor on the LOG's own liftoff (first throttle above idle): the tracker log's
        # sim_time runs 4-5 s behind events.csv (R0276-R0279, reviewer 2026-09-16), so the
        # events clock cannot index this file. Falls back to the file start.
        t0 = t[np.argmax(d['u2'] > 0.10)] if (d['u2'] > 0.10).any() else t[0]
        m = (t >= t0) & (t <= t0 + window)
        if m.sum() < 5:
            res[i] = None
            continue
        yaw = _yaw(d['pose_qw'][m], d['pose_qx'][m], d['pose_qy'][m], d['pose_qz'][m])
        res[i] = {'excursion_deg': float(np.max(np.abs(yaw - yaw[0]))),
                  'u3_saturated_frac': float(np.mean(np.abs(d['u3'][m]) > 0.95)),
                  'samples': int(m.sum()), 't_takeoff': float(t0)}
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+', help='run ids (R####), run paths, or a logs root with --stamp')
    ap.add_argument('--window', type=float, default=15.0)
    ap.add_argument('--stamp', default=None, help='rig logs: timestamp prefix of one flight')
    a = ap.parse_args()
    print(f"{'run':38s} {'drone':>5s} {'excursion':>10s} {'u3 sat':>7s} {'abort':>6s}")
    for spec in a.runs:
        path = spec if os.path.isdir(spec) else resolve(spec)
        ab = aborted(path) if a.stamp is None else None
        for i, r in excursions(path, a.window, a.stamp).items():
            name = os.path.basename(path.rstrip('/')) if a.stamp is None else f'rig {a.stamp}'
            if r is None:
                print(f"{name:38s} {i:5d} {'no data':>10s}")
                continue
            print(f"{name:38s} {i:5d} {r['excursion_deg']:9.1f}° {r['u3_saturated_frac']:7.2f} "
                  f"{('yes' if ab else 'no') if ab is not None else '-':>6s}")


if __name__ == '__main__':
    main()
