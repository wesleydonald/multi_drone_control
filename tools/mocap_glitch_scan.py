#!/usr/bin/env python3
"""
tools/mocap_glitch_scan.py -- how often Motive glitches the ring's pose, or a magnet marker, in our rig bags
(Wesley 21 Oct, on the magnet-marker check's critic round 2; card 2026-10-21_tip_detect rev 3).

    tools/mocap_glitch_scan.py results/rig/*/*/           # every bag (directories without a bag are skipped)

Airborne samples only (z > 0.3 m). Ring (/payload/motion_capture_state): departures of roll or pitch from a 0.2 s
centred median over 3 deg (a glitch jumps and comes back; real rocking is smooth), the largest frame-to-frame turn,
stamp gaps over 30 ms and the longest run of bit-identical poses (a lost body repeats its last pose). Marker
(/magnet_tip_pose and /drone_<i>/magnet_tip_state): departures from a 0.2 s median over 2 cm, the largest frame step
and the longest frozen run. Prints one line per bag and the totals.
"""
import argparse
import os
import re

import numpy as np
from scipy.ndimage import median_filter


def read(bag, want):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    topics = [t for t in types if want(t)]
    if not topics:
        return {}
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    cls = {t: get_message(types[t]) for t in topics}
    out = {t: [] for t in topics}
    while r.has_next():
        topic, data, ts = r.read_next()
        m = deserialize_message(data, cls[topic])
        p, o = m.pose.position, m.pose.orientation
        out[topic].append((ts * 1e-9, p.x, p.y, p.z, o.w, o.x, o.y, o.z))
    return {k: np.array(sorted(v)) for k, v in out.items() if len(v) > 50}


def runs(mask):
    """[(start, length)] of the True runs."""
    d = np.diff(np.r_[0, mask.astype(int), 0])
    s, e = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    return list(zip(s, e - s))


def ring(A):
    t, q = A[:, 0], A[:, 4:8] / np.linalg.norm(A[:, 4:8], axis=1, keepdims=True)
    air = A[:, 3] > 0.3
    w, x, y, z = q.T
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1, 1)))
    tilt = np.degrees(np.arccos(np.clip(np.cos(np.radians(roll)) * np.cos(np.radians(pitch)), -1, 1)))
    dev = np.maximum(np.abs(roll - median_filter(roll, 21, mode='nearest')),
                     np.abs(pitch - median_filter(pitch, 21, mode='nearest')))
    step = np.r_[0.0, np.degrees(2 * np.arccos(np.abs(np.sum(q[1:] * q[:-1], axis=1)).clip(0, 1)))]
    frozen = np.r_[False, np.all(A[1:, 1:8] == A[:-1, 1:8], axis=1)]
    level = air & (tilt < 20.0)
    dt = np.diff(t)
    return {'air_s': float(air.sum() * np.median(dt)), 'level_s': float(level.sum() * np.median(dt)),
            'dev_runs_level': [n for s, n in runs(level & (dev > 3.0))],
            'dev_runs_other': [n for s, n in runs(air & ~level & (dev > 3.0))],
            'step_level': float(step[level].max()) if level.any() else 0.0,
            'frozen_level': max([n for s, n in runs(level & frozen)] or [0]),
            'frozen_other': max([n for s, n in runs(air & ~level & frozen)] or [0]),
            'gaps': int(np.sum(dt[air[1:]] > 0.03))}


def marker(A):
    t, p = A[:, 0], A[:, 1:4]
    air = p[:, 2] > 0.3
    med = np.stack([median_filter(p[:, k], 21, mode='nearest') for k in range(3)], 1)
    dev = np.linalg.norm(p - med, axis=1)
    step = np.r_[0.0, np.linalg.norm(np.diff(p, axis=0), axis=1)]
    frozen = np.r_[False, np.all(A[1:, 1:4] == A[:-1, 1:4], axis=1)]
    return {'air_s': float(air.sum() * np.median(np.diff(t))),
            'dev_runs': [(n, float(dev[s:s + n].max())) for s, n in runs(air & (dev > 0.02))],
            'step': float(step[air].max()) if air.any() else 0.0,
            'frozen': max([n for s, n in runs(air & frozen)] or [0])}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    a = ap.parse_args()
    R, M = [], []
    for bag in a.bags:
        bag = bag.rstrip('/')
        if not os.path.exists(os.path.join(bag, 'metadata.yaml')):
            continue
        try:
            D = read(bag, lambda t: t == '/payload/motion_capture_state' or t == '/magnet_tip_pose'
                     or re.fullmatch(r'/drone_\d+/magnet_tip_state', t))
        except Exception as ex:  # noqa: BLE001
            print(f'{bag}: unreadable ({ex})')
            continue
        name = os.path.relpath(bag)
        if '/payload/motion_capture_state' in D:
            r = ring(D.pop('/payload/motion_capture_state'))
            if r['air_s'] > 0.5:
                R.append(r)
                print(f'{name}: ring airborne {r["air_s"]:.1f} s (level {r["level_s"]:.1f}): departures > 3 deg '
                      f'level {len(r["dev_runs_level"])} / tilted {len(r["dev_runs_other"])}, largest level step '
                      f'{r["step_level"]:.1f} deg, frozen run level {r["frozen_level"]} / tilted {r["frozen_other"]}, '
                      f'gaps {r["gaps"]}', flush=True)
        for topic, A in D.items():
            m = marker(A)
            if m['air_s'] > 0.5:
                M.append(m)
                print(f'{name} {topic}: marker airborne {m["air_s"]:.1f} s: departures > 2 cm {len(m["dev_runs"])} '
                      f'(longest {max([n for n, _ in m["dev_runs"]] or [0])} samples), largest step '
                      f'{100 * m["step"]:.1f} cm, frozen run {m["frozen"]}', flush=True)
    print(f'\nring: {len(R)} bags, {sum(r["air_s"] for r in R) / 60:.1f} min airborne, '
          f'{sum(r["level_s"] for r in R) / 60:.1f} min level (tilt < 20 deg); level flight: '
          f'{sum(len(r["dev_runs_level"]) for r in R)} departures > 3 deg, largest step '
          f'{max([r["step_level"] for r in R] or [0]):.1f} deg, longest frozen run '
          f'{max([r["frozen_level"] for r in R] or [0])}; tilted: {sum(len(r["dev_runs_other"]) for r in R)} departures')
    print(f'markers: {len(M)} streams, {sum(m["air_s"] for m in M) / 60:.1f} min airborne; jumps over 0.3 m in '
          f'{sum(1 for m in M if m["step"] > 0.3)}; departures > 2 cm lasting >= 3 samples in '
          f'{sum(1 for m in M if any(n >= 3 for n, _ in m["dev_runs"]))}')


if __name__ == '__main__':
    main()
