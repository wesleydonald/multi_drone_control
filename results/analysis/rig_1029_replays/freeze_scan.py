#!/usr/bin/env python3
"""Motive streams that hold one value for several samples (the stamp still advancing) and then jump: per topic the
share of repeated samples, the longest hold, and how far the first new sample lands from the held one.

    python3 freeze_scan.py sq5_2 k1_4 ...
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import OUT, pose_array, read_bag  # noqa: E402

TOPICS = ['/magnet_tip_pose', '/tejen/magnet_tip_pose', '/drone_3/motion_capture_state', '/payload/motion_capture_state',
          '/pickup/motion_capture_state', '/drone_0/motion_capture_state']


def scan(a):
    same = np.all(np.diff(a[:, 1:4], axis=0) == 0, axis=1)
    holds, c, start = [], 0, None
    for i, s in enumerate(same):
        if s:
            start = i if c == 0 else start
            c += 1
        elif c:
            holds.append((start, i + 1, c))
            c = 0
    jumps = [float(np.linalg.norm(a[e, 1:3] - a[s, 1:3])) for s, e, _ in holds if e < len(a)]
    dur = [float(a[e, 0] - a[s, 0]) for s, e, _ in holds if e < len(a)]
    return dict(n=len(a), repeated_frac=float(same.mean()), holds=len(holds),
                hold_s_median=float(np.median(dur)) if dur else 0.0, hold_s_max=float(max(dur)) if dur else 0.0,
                jump_cm_median=100 * float(np.median(jumps)) if jumps else 0.0,
                jump_cm_max=100 * float(max(jumps)) if jumps else 0.0)


def main():
    res = {}
    for run in sys.argv[1:]:
        b = read_bag(run, TOPICS)
        res[run] = {}
        for tp in TOPICS:
            if not b[tp]:
                continue
            res[run][tp] = scan(pose_array(b[tp]))
            r = res[run][tp]
            print(f"{run:7s} {tp:32s} n {r['n']:6d} repeated {100 * r['repeated_frac']:5.1f} %  holds {r['holds']:4d} "
                  f"median {r['hold_s_median']:.3f} s max {r['hold_s_max']:.3f} s  jump median {r['jump_cm_median']:.1f} cm "
                  f"max {r['jump_cm_max']:.1f} cm")
    with open(os.path.join(OUT, 'freeze_scan.json'), 'w') as f:
        json.dump(res, f, indent=1)


if __name__ == '__main__':
    main()
