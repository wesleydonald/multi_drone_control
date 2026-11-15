"""feeds.py <attempt dir> ...: ring_bridge's outputs as received by an independent recorder (runs_disable/<attempt>/feeds,
system-clock receive times; ROS-time by the recorded /clock), against the backend's disable time.

Per topic: the largest receive gap over the run and the largest within [disable - 2 s, disable]; for the ring pose
array, the age the backend's 0.5 s ring-pose check would have seen at the disable (time since the last pose received
before it), and the header-stamp gap (ring_bridge's own publish clock: a stall inside ring_bridge shows there too)."""
import glob
import os
import re
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ['/fake_payload/pose_array', '/fake_obstacles/drone_0/state', '/fake_obstacles/drone_1/state',
          '/fake_obstacles/drone_2/state', '/payload/motion_capture_state', '/fake_payload/committed_trajectory',
          '/fake_obstacles/drone_0/committed_trajectory']


def load(db):
    con = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    topics = {n: (i, t) for i, n, t in con.execute('select id, name, type from topics')}
    out = {}
    for name, (tid, ttype) in topics.items():
        rows = con.execute('select timestamp, data from messages where topic_id=? order by timestamp', (tid,)).fetchall()
        out[name] = (ttype, rows)
    return out


def main(dirs):
    for d in dirs:
        dbs = glob.glob(os.path.join(d, 'feeds', '*.db3'))
        if not dbs:
            print(d, 'no feeds bag')
            continue
        bag = load(dbs[0])
        # wall (record) -> ROS (the /clock value received at that wall time)
        ctype, crows = bag['/clock']
        cls = get_message(ctype)
        cw = np.array([r[0] * 1e-9 for r in crows])
        cr = np.array([(lambda m: m.clock.sec + 1e-9 * m.clock.nanosec)(deserialize_message(r[1], cls)) for r in crows])
        to_ros = lambda w: np.interp(w, cw, cr)
        log = open(os.path.join(d, 'backend.log'), errors='replace').read()
        m = re.search(r'Backend disabled at ROS ([0-9.]+): (\S+?);', log)
        t_dis = float(m.group(1)) if m else None
        ev = open(os.path.join(d, 'events.txt')).readline().split()
        t0 = float(ev[0])
        print(f'== {os.path.basename(d)}: disable {"at +%.3f %s" % (t_dis - t0, m.group(2)) if m else "none"}')
        for name in TOPICS:
            if name not in bag:
                continue
            ttype, rows = bag[name]
            w = np.array([r[0] * 1e-9 for r in rows])
            ros = to_ros(w)
            sel = ros >= t0 - 1.0
            gaps = np.diff(ros[sel])
            tt = ros[sel][1:]
            line = f'  {name:44s} n={sel.sum():5d} max gap {gaps.max():.3f} s at +{tt[np.argmax(gaps)] - t0:.2f}'
            if t_dis is not None:
                win = (tt <= t_dis) & (tt >= t_dis - 2.0)
                before = ros[ros <= t_dis]
                line += (f'; within 2 s before the disable max {gaps[win].max() if win.any() else float("nan"):.3f};'
                         f' last receipt before it {t_dis - before[-1]:.3f} s earlier' if before.size else '')
            if name == '/fake_payload/pose_array':
                cls2 = get_message(ttype)
                hs = np.array([(lambda mm: mm.header.stamp.sec + 1e-9 * mm.header.stamp.nanosec)(
                    deserialize_message(r[1], cls2)) for r in rows])[sel]
                hg = np.diff(hs)
                line += f'; header-stamp max gap {hg.max():.3f} s at +{hs[1:][np.argmax(hg)] - t0:.2f}'
                big = np.where(gaps > 0.25)[0]
                line += '; receive gaps > 0.25 s at ' + ', '.join(f'+{tt[k] - t0:.2f} ({gaps[k]:.2f})' for k in big[:8])
            print(line)


if __name__ == '__main__':
    main(sys.argv[1:] or sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                       'runs_disable', 'attempt*'))))
