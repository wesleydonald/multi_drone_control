#!/usr/bin/env python3
"""tools/tilt_replay.py -- replay the QUAD4 tilt cut offline on recorded mocap (card 2026-10-11_quad4_tilt_cut, M1).

    python3 tools/tilt_replay.py results/rig/2026-10-08/r302_2 [more bag dirs or .db3 ...]
        [--topic /drone_3/motion_capture_state] [--cut 60] [--samples 3] [--glitch 45] [--slow 30] [--slow-s 0.5]

Reads a rosbag2 sqlite bag directly (no ros2 bag play) and feeds every pose of the topic to the same rule the tracker
runs (utility_objects.tilt_cut), active over the whole bag (stricter than in flight). Per bag: poses, max tilt and its
p99.9, single-pose attitude steps over 30 and 45 deg, the speed p99.9 and max, and whether the cut (and the optional
slow tier) would have fired, with the time. Needs the workspace sourced (interfaces).
"""
import argparse
import glob
import math
import os
import sqlite3
import sys

import numpy as np


def bag_files(paths):
    out = []
    for p in paths:
        out += sorted(glob.glob(os.path.join(p, '*.db3'))) if os.path.isdir(p) else [p]
    return out


def poses(db3, topic):
    """(t_s, (w, x, y, z), speed) per message of topic."""
    from rclpy.serialization import deserialize_message
    from interfaces.msg import MotionCaptureState
    con = sqlite3.connect(db3)
    row = con.execute('select id from topics where name = ?', (topic,)).fetchone()
    if row is None:
        return []
    out = []
    for ts, data in con.execute('select timestamp, data from messages where topic_id = ? order by timestamp',
                                (row[0],)):
        m = deserialize_message(data, MotionCaptureState)
        o, v = m.pose.orientation, m.twist.linear
        out.append((ts * 1e-9, (o.w, o.x, o.y, o.z), math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)))
    return out


def replay(samples, cut=60.0, n=3, glitch=45.0, slow=30.0, slow_s=0.5):
    from utility_objects.safety import tilt_deg_from_quat
    from utility_objects.tilt_cut import TiltCutConfig, TiltCutRule, attitude_step_deg
    rules = {'cut': TiltCutRule(TiltCutConfig(cut_deg=cut, samples=n, glitch_step_deg=glitch)),
             'slow': TiltCutRule(TiltCutConfig(cut_deg=cut, samples=n, glitch_step_deg=glitch, slow_deg=slow,
                                               slow_s=slow_s))}
    for r in rules.values():
        r.takeoff()
    fired = {}
    tilts, steps, speeds = [], [], []
    prev = None
    t0 = samples[0][0] if samples else 0.0
    for t, q, speed in samples:
        tilts.append(tilt_deg_from_quat(q))
        speeds.append(speed)
        if prev is not None:
            steps.append(attitude_step_deg(prev, q))
        prev = q
        for name, r in rules.items():
            why = r.step(t, q)
            if why and name not in fired:
                fired[name] = (why, t - t0)
    tilts, steps, speeds = (np.array(x, float) for x in (tilts, steps, speeds))
    return {'poses': len(samples), 'duration_s': (samples[-1][0] - t0) if samples else 0.0,
            'max_tilt': float(np.nanmax(tilts)) if len(tilts) else math.nan,
            'p999_tilt': float(np.nanpercentile(tilts, 99.9)) if len(tilts) else math.nan,
            'steps_over_30': int(np.sum(steps > 30.0)), 'steps_over_45': int(np.sum(steps > 45.0)),
            'speed_p999': float(np.nanpercentile(speeds, 99.9)) if len(speeds) else math.nan,
            'speed_max': float(np.nanmax(speeds)) if len(speeds) else math.nan, 'fired': fired}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--topic', default='/drone_3/motion_capture_state')
    ap.add_argument('--cut', type=float, default=60.0)
    ap.add_argument('--samples', type=int, default=3)
    ap.add_argument('--glitch', type=float, default=45.0)
    ap.add_argument('--slow', type=float, default=30.0)
    ap.add_argument('--slow-s', type=float, default=0.5)
    a = ap.parse_args()
    total = 0
    print(f'{"bag":44s} {"poses":>6s} {"dur s":>6s} {"max":>6s} {"p99.9":>6s} {">30":>4s} {">45":>4s} '
          f'{"v p99.9":>7s} {"v max":>6s}  cut / slow')
    for db in bag_files(a.bags):
        s = poses(db, a.topic)
        if not s:
            continue
        r = replay(s, a.cut, a.samples, a.glitch, a.slow, a.slow_s)
        total += r['poses']
        fired = ', '.join(f'{k} {w} at {t:.1f} s' for k, (w, t) in r['fired'].items()) or 'none'
        print(f'{os.path.relpath(db)[-44:]:44s} {r["poses"]:6d} {r["duration_s"]:6.1f} {r["max_tilt"]:6.1f} '
              f'{r["p999_tilt"]:6.1f} {r["steps_over_30"]:4d} {r["steps_over_45"]:4d} {r["speed_p999"]:7.2f} '
              f'{r["speed_max"]:6.2f}  {fired}')
    print(f'{total} poses')
    return 0


if __name__ == '__main__':
    sys.exit(main())
