#!/usr/bin/env python3
"""His MATCH_VELOCITY -> DESCEND gate replayed from a rig bag, as flown and with other settings.

    python3 tools/match_gate_replay.py results/rig/2026-10-15/a3_2 [more runs]
    python3 tools/match_gate_replay.py RUN --gate 0.05 0.15 1.0 --gate 0.05 0.15 0.5

Contact mode's gate (online_join_planner): the measured tip within xy of the plate point AND QUAD4's xy velocity
within speed of the target's, both unbroken for the dwell. Rebuilt as the stack builds it: the ring bridge's 50 Hz
timer takes the latest ring pose (sample and hold), puts the plate point through the full ring attitude and
finite-differences it for the target velocity; QUAD4's velocity is its mocap twist; the gate is checked at his
30 Hz. On 15 Oct this reproduced a1's three descents (13.2, 3.9, 1.0 s into MATCH); it is exact to about one
check (33 ms), so a gate that only just holds (a3_2) can go either way.
"""
import argparse
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as Rot

PLATE = (0.013, 0.210, 0.024)       # plate 9's point in the ring frame (the sheet's X, Y, W)
BRIDGE_DT, HIS_DT = 0.02, 1.0 / 30.0


def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    ring, q4, tip, phases = [], [], [], []
    while r.has_next():
        tp, raw, t = r.read_next()
        if tp not in ('/payload/motion_capture_state', '/drone_3/motion_capture_state', '/magnet_tip_pose',
                      '/join_planner/phase'):
            continue
        m = deserialize_message(raw, get_message(types[tp]))
        if tp == '/payload/motion_capture_state':
            p, q = m.pose.position, m.pose.orientation
            ring.append([t * 1e-9, p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        elif tp == '/drone_3/motion_capture_state':
            q4.append([t * 1e-9, m.twist.linear.x, m.twist.linear.y])
        elif tp == '/magnet_tip_pose':
            tip.append([t * 1e-9, m.pose.position.x, m.pose.position.y])
        elif not phases or phases[-1][1] != m.data:
            phases.append((t * 1e-9, m.data))
    return np.array(ring), np.array(q4), np.array(tip), phases


def hold(arr, tq):
    return arr[np.clip(np.searchsorted(arr[:, 0], tq, side='right') - 1, 0, None)]


def replay(ring, q4, tip, a, b, plate, gates, zero_twist=False):
    tb = np.arange(a - 0.1, b, BRIDGE_DT)
    rr = hold(ring, tb)
    p = rr[:, 1:4] + Rot.from_quat(rr[:, 4:8]).apply(plate)
    vt = np.vstack([[0.0, 0.0, 0.0], np.diff(p, axis=0) / BRIDGE_DT])
    if zero_twist:                     # attach_twist:=zero: the gate compares QUAD4's own velocity
        vt = np.zeros_like(vt)
    th = np.arange(a, b + 0.05, HIS_DT)          # past the phase's end: the check that passed it
    k = np.clip(np.searchsorted(tb, th, side='right') - 1, 0, None)
    err = np.linalg.norm(hold(tip, th)[:, 1:3] - p[k, :2], axis=1)
    rel = np.linalg.norm(hold(q4, th)[:, 1:3] - vt[k, :2], axis=1)
    out = []
    for xy, speed, dwell in gates:
        both = (err < xy) & (rel < speed)
        run, first, passes, longest = 0, None, 0, 0
        for j, x in enumerate(both):
            run = run + 1 if x else 0
            longest = max(longest, run)
            if (run - 1) * HIS_DT >= dwell - 1e-9:     # his condition_true_for: from the first true check
                passes += 1
                run = 0
                first = th[j] - a if first is None else first
        out.append((xy, speed, dwell, first, passes, max(0, longest - 1) * HIS_DT, both.mean()))
    return out, float(np.sqrt(np.mean(np.sum(vt[k, :2] ** 2, axis=1))))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--gate', nargs=3, type=float, action='append', metavar=('XY', 'SPEED', 'DWELL'),
                    help='a gate to replay (m, m/s, s); default his rig gate 0.05 0.15 1.0 and three looser ones')
    ap.add_argument('--plate', nargs=3, type=float, default=PLATE, metavar=('X', 'Y', 'Z'))
    ap.add_argument('--zero-twist', action='store_true', help='replay with a zero target velocity (attach_twist:=zero)')
    a = ap.parse_args(argv)
    gates = a.gate or [(0.05, 0.15, 1.0), (0.05, 0.15, 0.5), (0.05, 0.20, 1.0), (0.07, 0.15, 1.0)]
    for run in a.runs:
        ring, q4, tip, phases = read(run)
        print(f'== {run}')
        for i, (t0, name) in enumerate(phases):
            if name != 'MATCH_VELOCITY':
                continue
            t1 = phases[i + 1][0] if i + 1 < len(phases) else t0 + 5.0
            rows, vrms = replay(ring, q4, tip, t0, t1, a.plate, gates, a.zero_twist)
            print(f'  MATCH {t1 - t0:5.1f} s (then {phases[i + 1][1] if i + 1 < len(phases) else "end"}), '
                  f'target speed rms {vrms:.3f} m/s')
            for xy, speed, dwell, first, passes, longest, frac in rows:
                when = 'never' if first is None else f'{first:.1f} s'
                print(f'    xy < {100 * xy:.0f} cm, speed < {speed:.2f}, dwell {dwell:.1f} s: first pass {when}, '
                      f'{passes} passes, both {100 * frac:.0f} %, longest {longest:.2f} s')
    return 0


if __name__ == '__main__':
    sys.exit(main())
