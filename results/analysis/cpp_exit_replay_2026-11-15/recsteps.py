"""The recorded tick-to-tick reference steps of each 13 Nov flight's own exit (his Python clearance and return, as
flown): each /join_planner/reference publication's sample 0 against the previous publication evaluated at the new
stamp (linear between its samples, as advance_reference_window), over [CLEAR_DROP_ZONE entry, the recorded LAND).
-> flights/recsteps.json: per flight p95 and max of |dp| m, |dv| m/s, |da| m/s^2, overall and at the RETURN and
AT_START phase switches."""
import json
import os
import sqlite3

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
R = '/home/wesley/multi_drone_control/results/rig/2026-11-13'
FL = ['fm9h_2', 'fm9hc_10', 'fm9hc_11', 'fm9h8_13', 'fm9h8_14', 'fm9h8_5', 'fm9hc', 'fm9hc_8']


def window(m):
    tf = np.array([p.time_from_start.sec + 1e-9 * p.time_from_start.nanosec for p in m.points])
    P = np.array([[p.transforms[0].translation.x, p.transforms[0].translation.y, p.transforms[0].translation.z]
                  for p in m.points])
    V = np.array([[p.velocities[0].linear.x, p.velocities[0].linear.y, p.velocities[0].linear.z] for p in m.points])
    A = np.array([[p.accelerations[0].linear.x, p.accelerations[0].linear.y, p.accelerations[0].linear.z]
                  if p.accelerations else [0.0, 0.0, 0.0] for p in m.points])
    return m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec, tf, P, V, A


def at(w, t):
    st, tf, P, V, A = w
    tau = min(max(t - st, tf[0]), tf[-1])
    return [np.array([np.interp(tau, tf, X[:, j]) for j in range(3)]) for X in (P, V, A)]


out = {}
for f in FL:
    j = json.load(open(os.path.join(HERE, 'flights', f + '.json')))
    con = sqlite3.connect(f'file:{R}/{f}/{f}_0.db3?mode=ro', uri=True)
    tid, ttype = con.execute("select id, type from topics where name='/join_planner/reference'").fetchone()
    cls = get_message(ttype)
    ws = [window(deserialize_message(d, cls)) for (d,) in con.execute(
        'select data from messages where topic_id=? and timestamp>=? and timestamp<=? order by timestamp',
        (tid, int((j['t_clear'] - 0.5) * 1e9), int(j['t_land'] * 1e9)))]
    steps, times = [], []
    for a, b in zip(ws[:-1], ws[1:]):
        p, v, acc = at(a, b[0])
        steps.append([np.linalg.norm(b[2][0] - p), np.linalg.norm(b[3][0] - v), np.linalg.norm(b[4][0] - acc)])
        times.append(b[0] - j['t_clear'])
    steps, times = np.array(steps), np.array(times)
    m = times >= 0.0
    t_ret = j['t_ret'] - j['t_clear']
    k_ret = int(np.argmin(np.abs(times - t_ret)))
    out[f] = dict(n=int(m.sum()), period_ms=float(1e3 * np.median(np.diff([w[0] for w in ws]))),
                  p95=[round(float(x), 4) for x in np.percentile(steps[m], 95, axis=0)],
                  max=[round(float(x), 4) for x in np.max(steps[m], axis=0)],
                  at_return=[round(float(x), 4) for x in np.max(steps[max(0, k_ret - 1):k_ret + 2], axis=0)])
    print(f, out[f])
json.dump(out, open(os.path.join(HERE, 'flights', 'recsteps.json'), 'w'), indent=1)
