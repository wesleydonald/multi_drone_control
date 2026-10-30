"""Extract the attach-relevant topics of a rig bag to an npz cache (read-only on the repo)."""
import json
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

OUT = os.path.dirname(os.path.abspath(__file__))


def main(bagdir, tag):
    db = [os.path.join(bagdir, f) for f in os.listdir(bagdir) if f.endswith('.db3')][0]
    con = sqlite3.connect(db)
    cur = con.cursor()
    tops = {n: (i, t) for i, n, t in cur.execute('select id,name,type from topics')}
    data = {}

    def poses(name, quat):
        if name not in tops:
            return np.zeros((0, 9 if quat else 5))
        i, ty = tops[name]
        cls = get_message(ty)
        rows = []
        for ts, d in cur.execute('select timestamp,data from messages where topic_id=? order by timestamp', (i,)):
            m = deserialize_message(d, cls)
            h = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            p, q = m.pose.position, m.pose.orientation
            rows.append((ts * 1e-9, h, p.x, p.y, p.z) + ((q.x, q.y, q.z, q.w) if quat else ()))
        return np.array(rows)

    data['tip'] = poses('/magnet_tip_pose', False)
    data['ring'] = poses('/payload/motion_capture_state', True)
    data['body'] = poses('/drone_3/motion_capture_state', True)
    strings = {}
    for name in ('/join_planner/phase', '/magnet/attach_proof', '/magnet/contact', '/drone_3/magnet_coil',
                 '/magnet/command', '/join_planner/proof_lift_complete', '/magnet/object_attached',
                 '/join_planner/give_up', '/drone_3/take_back', '/tejen/magnet/command'):
        if name not in tops:
            continue
        i, ty = tops[name]
        cls = get_message(ty)
        strings[name] = [(ts * 1e-9, deserialize_message(d, cls).data)
                         for ts, d in cur.execute('select timestamp,data from messages where topic_id=? order by '
                                                  'timestamp', (i,))]
    np.savez(os.path.join(OUT, f'{tag}.npz'), **data)
    with open(os.path.join(OUT, f'{tag}_strings.json'), 'w') as f:
        json.dump({k: [(t, v if isinstance(v, str) else bool(v)) for t, v in s] for k, s in strings.items()}, f)
    print(tag, {k: len(v) for k, v in data.items()}, {k: len(v) for k, v in strings.items()})


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
