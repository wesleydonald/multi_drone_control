#!/usr/bin/env python3
"""Extract the motion-capture poses, QUAD4's magnet tip, his reference, phase and magnet topics of the 13 Nov rig
flights into cache/<flight>.npz (full rate, bag receive time in s).

Reads the rosbag2 sqlite file directly (fm9h8_4 has no metadata.yaml, so rosbag2_py cannot open it).
usage: python3 extract.py [flight ...]      (default: all 14 C++ flights)
needs: source ~/ros2_humble/install/setup.bash && source install/setup.bash"""
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..'))
RIG = os.path.join(ROOT, 'results', 'rig', '2026-11-13')
FLIGHTS = ['fm9h8_3', 'fm9h8_4', 'fm9h8_5', 'fm9hc', 'fm9hc_5', 'fm9hc_8', 'fm9h8_7', 'fm9h8_10', 'fm9h8_13',
           'fm9h8_14', 'fm9hc_9', 'fm9hc_10', 'fm9h_2', 'fm9hc_11']
POSE = {'/drone_0/motion_capture_state': 'd0', '/drone_1/motion_capture_state': 'd1',
        '/drone_2/motion_capture_state': 'd2', '/drone_3/motion_capture_state': 'd3',
        '/payload/motion_capture_state': 'ring', '/pickup/motion_capture_state': 'obj',
        '/magnet_tip_pose': 'tip'}
BOOL = {'/magnet/object_attached': 'obj_att', '/tejen/object_attached': 'tj_obj_att', '/magnet/contact': 'contact'}
STR = {'/join_planner/phase': 'phase', '/drone_3/magnet_coil': 'coil', '/tejen/magnet/command': 'tj_mag_cmd'}
REF = '/join_planner/reference'


def extract(name):
    db = os.path.join(RIG, name, f'{name}_0.db3')
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    topics = {tid: (tn, tt) for tid, tn, tt in con.execute('select id, name, type from topics')}
    want = set(POSE) | set(BOOL) | set(STR) | {REF}
    ids = {tid: tn for tid, (tn, tt) in topics.items() if tn in want}
    cls = {tn: get_message(tt) for tid, (tn, tt) in topics.items() if tn in want}
    rows = {k: [] for k in list(POSE.values()) + ['ref']}
    ev = {k: [] for k in list(BOOL.values()) + list(STR.values())}
    q = f'select topic_id, timestamp, data from messages where topic_id in ({",".join(map(str, ids))}) order by timestamp'
    for tid, ts, data in con.execute(q):
        tn = ids[tid]
        m = deserialize_message(data, cls[tn])
        s = ts * 1e-9
        if tn in POSE:
            p, o = m.pose.position, m.pose.orientation
            rows[POSE[tn]].append((s, p.x, p.y, p.z, o.w, o.x, o.y, o.z))
        elif tn in BOOL:
            ev[BOOL[tn]].append((s, str(int(m.data))))
        elif tn in STR:
            ev[STR[tn]].append((s, m.data))
        elif tn == REF and m.points and m.points[0].transforms:
            p = m.points[0].transforms[0].translation
            rows['ref'].append((s, p.x, p.y, p.z))
    out = {}
    for k, v in rows.items():
        out[k] = np.array(v, dtype=float) if v else np.zeros((0, 8 if k != 'ref' else 4))
    for k, v in ev.items():
        # keep changes only (phase is published at ~30 Hz)
        keep = [r for i, r in enumerate(v) if i == 0 or r[1] != v[i - 1][1]]
        out[k + '_t'] = np.array([r[0] for r in keep], dtype=float)
        out[k + '_v'] = np.array([r[1] for r in keep], dtype=object).astype(str)
    os.makedirs(os.path.join(HERE, 'cache'), exist_ok=True)
    np.savez_compressed(os.path.join(HERE, 'cache', f'{name}.npz'), **out)
    print(name, {k: len(v) for k, v in rows.items()}, 'phases', len(out['phase_t']))


if __name__ == '__main__':
    for n in (sys.argv[1:] or FLIGHTS):
        extract(n)
