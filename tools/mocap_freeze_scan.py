#!/usr/bin/env python3
"""tools/mocap_freeze_scan.py -- mocap streams that repeat one pose while the stamps advance, then jump (Motive holds a
rigid body's last pose while it has lost its markers; RIG-1029-sq5_2: the magnet tip, 13.4 % of samples, ~0.2 s holds).

    python3 tools/mocap_freeze_scan.py results/rig/2026-10-30/sq6 [more bag dirs...]

Per pose topic in the bag: the share of repeated samples, the holds (count, median, longest) and how far the first new
sample lands from the held one. Prints only.
"""
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

TOPICS = ('/magnet_tip_pose', '/payload/motion_capture_state', '/pickup/motion_capture_state') + tuple(
    f'/drone_{i}/motion_capture_state' for i in range(4))


def read(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {}
    while r.has_next():
        topic, raw, t = r.read_next()
        if topic in TOPICS:
            p = deserialize_message(raw, get_message(types[topic])).pose.position
            out.setdefault(topic, []).append((t * 1e-9, p.x, p.y, p.z))
    return {k: np.array(v) for k, v in out.items()}


def scan(a):
    same = np.all(np.diff(a[:, 1:], axis=0) == 0, axis=1)
    holds, run = [], 0
    for i, s in enumerate(same):
        if s:
            run += 1
        elif run:
            holds.append((i + 1, run))       # a[i + 1] is the first new sample after a[i + 1 - run - 1 .. i]
            run = 0
    dur = [a[i - 1, 0] - a[i - 1 - k, 0] for i, k in holds]
    jump = [float(np.linalg.norm(a[i, 1:3] - a[i - 1, 1:3])) for i, k in holds]
    return same.mean(), len(holds), dur, jump


def main(bags):
    for bag in bags:
        print(f'== {bag}')
        for topic, a in sorted(read(bag).items()):
            if len(a) < 3:
                continue
            frac, n, dur, jump = scan(a)
            rate = (len(a) - 1) / max(a[-1, 0] - a[0, 0], 1e-9)
            print(f'  {topic:32s} {rate:5.0f} Hz  repeated {100 * frac:5.1f} %  holds {n:4d}' + (
                f'  median {np.median(dur):.2f} s  longest {max(dur):.2f} s  jump median {100 * np.median(jump):.1f} cm'
                f'  max {100 * max(jump):.1f} cm' if n else ''))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
