#!/usr/bin/env python3
"""Read the rig bags this analysis needs: one dict of numpy arrays per flight (cached as .npz).

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    BAND_CACHE=<dir> nice -n 10 python3 results/analysis/attach_band_2026-11-10/read_flights.py   # fills the cache

Topics (what Te-Jen's planner reads on the rig, partner_mission.launch.py real path):
  tip   /tejen/magnet_tip_pose          his measured tip (body 24 via partner_rig_state)       t, x, y, z
  body  /drone_3/motion_capture_state   QUAD4's pose and the velocity his gate uses (twist)    t, x, y, z, vx, vy, vz
  ring  /payload/motion_capture_state   the ring (the plate point = ring pose x contact target) t, x, y, z, qx, qy, qz, qw
  att_v /fake_attachment_point/twist    the plate's velocity his gate subtracts (zero when attach_twist:=zero)
  ref   /join_planner/reference         his body reference (first sample)                       t, x, y, z
  phase /join_planner/phase             his phase                                                (t, name)
Times are the recorder's receive times.
"""
import glob
import math
import os
import pickle
import sqlite3
import sys

import numpy as np

RIG = os.path.expanduser('~/multi_drone_control/results/rig')
CACHE = os.environ.get('BAND_CACHE', '')
TOPICS = {'tip': '/tejen/magnet_tip_pose', 'tip_raw': '/magnet_tip_pose', 'body': '/drone_3/motion_capture_state',
          'ring': '/payload/motion_capture_state', 'att_v': '/fake_attachment_point/twist',
          'ref': '/join_planner/reference', 'phase': '/join_planner/phase'}

# (day, flight): the flights with a plate descent of QUAD4 (4-5 Nov: rig_joins_2026-11-04_05/joins.csv) and the two
# 29 Oct flights with 1 m cables that reached his MATCH (sq2_5: his default gates; sq5_2: 0.08 m, no hold, no debounce)
FLIGHTS = [('2026-10-29', 'sq2_5'), ('2026-10-29', 'sq5_2'),
           ('2026-11-04', 'sq6b'), ('2026-11-04', 'sq6m'), ('2026-11-04', 'nr1_2'), ('2026-11-04', 'nr9m'),
           ('2026-11-04', 'nr9m_4'), ('2026-11-04', 'nr9m_5'),
           ('2026-11-05', 'fm9y_2'), ('2026-11-05', 'nr1r_3'), ('2026-11-05', 'sq8_2'), ('2026-11-05', 'nr9o_2'),
           ('2026-11-05', 'nr9m')]


def _read(folder):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    out = {k: [] for k in TOPICS}
    for db in sorted(glob.glob(os.path.join(folder, '*.db3'))):
        c = sqlite3.connect(db)
        tops = {n: (i, ty) for i, n, ty in c.execute('select id,name,type from topics')}
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
                elif key == 'att_v':
                    v = m.twist.linear
                    out[key].append((t, v.x, v.y, v.z))
                elif key == 'body':
                    p, v = m.pose.position, m.twist.linear
                    out[key].append((t, p.x, p.y, p.z, v.x, v.y, v.z))
                elif key == 'ring':
                    p, q = m.pose.position, m.pose.orientation
                    out[key].append((t, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                else:
                    p = m.pose.position
                    out[key].append((t, p.x, p.y, p.z))
        c.close()
    arr = {}
    for k, v in out.items():
        if k == 'phase':
            arr[k] = v
        else:
            arr[k] = np.array(sorted(v)) if v else np.zeros((0, 4))
    return arr


def load(day, name):
    """The flight's arrays, from the cache when BAND_CACHE is set."""
    folder = os.path.join(RIG, day, name)
    if not CACHE:
        return _read(folder)
    fn = os.path.join(CACHE, f'{day}_{name}.pkl')
    if os.path.exists(fn):
        with open(fn, 'rb') as f:
            return pickle.load(f)
    arr = _read(folder)
    os.makedirs(CACHE, exist_ok=True)
    with open(fn + '.tmp', 'wb') as f:
        pickle.dump(arr, f)
    os.replace(fn + '.tmp', fn)
    return arr


if __name__ == '__main__':
    todo = FLIGHTS if len(sys.argv) < 2 else [tuple(a.split('/')) for a in sys.argv[1:]]
    for day, name in todo:
        a = load(day, name)
        print(day, name, {k: (len(v) if not isinstance(v, float) else v) for k, v in a.items()}, flush=True)
