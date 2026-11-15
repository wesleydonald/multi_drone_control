"""Per 13 Nov flight: the bag time of CLEAR_DROP_ZONE's entry (the replay's start), the bag offset, QUAD4's start xy
(TAKEOFF) and the recorded tracking error from then on (QUAD4's mocap minus its /join_planner/reference evaluated at the
mocap time: the latest published window, interpolated at t - stamp). Output flights/<f>.json + flights/<f>_err.csv."""
import csv
import json
import os
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

A = '/home/wesley/multi_drone_control/results/analysis/cpp_rig_2026-11-13'
R = '/home/wesley/multi_drone_control/results/rig/2026-11-13'
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'flights')
sys.path.insert(0, A)
from common import parse_log, phase_t  # noqa: E402

FL = sys.argv[1:] or ['fm9h_2', 'fm9hc_10', 'fm9hc_11', 'fm9h8_13', 'fm9h8_14', 'fm9h8_5', 'fm9hc', 'fm9hc_8']
TRAJ = {'fm9h_2': 'hover'}


def rows(con, topic, t0, t1):
    tid, ttype = con.execute('select id, type from topics where name=?', (topic,)).fetchone()
    cls = get_message(ttype)
    for ts, data in con.execute('select timestamp, data from messages where topic_id=? and timestamp>=? and '
                                'timestamp<=? order by timestamp', (tid, int(t0 * 1e9), int(t1 * 1e9))):
        yield ts * 1e-9, deserialize_message(data, cls)


for f in FL:
    L = parse_log(f)
    tk = phase_t(L, 'TAKEOFF')
    tdrop = phase_t(L, 'DROP_OBJECT', after=tk)
    tclear = phase_t(L, 'CLEAR_DROP_ZONE', after=tdrop)
    tret = phase_t(L, 'RETURN_TO_START', after=tclear)
    con = sqlite3.connect(f'file:{R}/{f}/{f}_0.db3?mode=ro', uri=True)
    b0, b1 = (x * 1e-9 for x in con.execute('select min(timestamp), max(timestamp) from messages').fetchone())
    s = np.load(os.path.join(A, 'series', f + '.npz'), allow_pickle=False)
    start_xy = s['d3'][int(np.argmin(np.abs(s['t'])))][:2]
    # the recorded error from 2 s before the clearance to the bag's end
    refs = [(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec,
             np.array([[p.transforms[0].translation.x, p.transforms[0].translation.y, p.transforms[0].translation.z]
                       for p in m.points]),
             np.array([[p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z]
                       for p in m.points]),
             np.array([p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec for p in m.points]))
            for _, m in rows(con, '/join_planner/reference', tclear - 3.0, b1)]
    rstamps = np.array([r[0] for r in refs])
    out = []
    for tb, m in rows(con, '/drone_3/motion_capture_state', tclear - 2.0, b1):
        tm = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        k = int(np.searchsorted(rstamps, tm, side='right')) - 1
        if k < 0:
            continue
        st, P, V, tf = refs[k]
        tau = tm - st
        if tau > tf[-1] or tau < -0.05:
            continue
        p = np.array([np.interp(tau, tf, P[:, j]) for j in range(3)])
        v = np.array([np.interp(tau, tf, V[:, j]) for j in range(3)])
        mp = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
        mv = np.array([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z])
        out.append([tm - tclear, *(mp - p), *(mv - v), tau])
    out = np.array(out)
    with open(os.path.join(D, f + '_err.csv'), 'w', newline='\n') as fh:
        fh.write('t_rel,ex,ey,ez,evx,evy,evz,ref_age\n')
        for r in out:
            fh.write(','.join(f'{x:.5f}' for x in r) + '\n')
    info = dict(flight=f, traj=TRAJ.get(f, 'circle' if 'hc' in f else 'fig8'), bag_start=b0, bag_end=b1,
                t_takeoff=tk, t_drop=tdrop, t_clear=tclear, t_ret=tret, clear_offset=tclear - b0,
                start_xy=[float(x) for x in start_xy], err_until_s=float(out[-1, 0]),
                err_p95_xyz_m=float(np.percentile(np.linalg.norm(out[out[:, 0] >= 0, 1:4], axis=1), 95)),
                err_p95_v_mps=float(np.percentile(np.linalg.norm(out[out[:, 0] >= 0, 4:7], axis=1), 95)))
    json.dump(info, open(os.path.join(D, f + '.json'), 'w'), indent=1)
    print(json.dumps(info))
