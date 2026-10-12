#!/usr/bin/env python3
"""tools/rate_latency.py -- the rig's command-to-motion latency: the body rates the trackers commanded against the
body rates the drones flew, from the bags (thesis plan P6; read-only, rosbag2_py, no node is started).

    source ~/ros2_humble/install/setup.bash; source install/setup.bash
    python3 tools/rate_latency.py [--dates 2026-10-07 2026-10-08] [--out DIR] [--cache DIR]

Commanded: every /drone_i/ELRSCommand sent armed (50 Hz), the roll, pitch and yaw sticks through the tracker's
Betaflight rate curve (rates_centre_deg, rates_max_deg, rates_expo from its params.json): [roll, pitch, yaw] rate
= [BF(ch0), BF(ch1), BF(-ch3)], the mapping the tracker's own model uses (tracker/dynamics.py w_dynamics). Held
between messages (the flight controller holds the last packet).
Measured: the body rate from the drone's mocap quaternions (body frame, central difference over the header stamps);
the bag's twist.angular is not used (rounded to 0.1 rad/s and low-passed on every flight to 8 Oct).
Clock: both on the bag's receive clock, i.e. the time the command left the tracker and the time the pose reached
the subscribers, so the latency is the loop as the controller sees it: radio, flight controller, motors, the
airframe, Motive and the mocap node. (Header stamps sit 0.3 ms before the receive time on 8 Oct.)
Window: the drone airborne (mocap z 0.15 m above its rest), armed, from TAKEOFF + 3 s to LAND or the first fleet
stop. Per drone and axis, on a 250 Hz grid with each signal's mean removed:
  xcorr_lag_ms   the lag of the peak correlation of measured(t + L) with commanded(t), L in 0-300 ms;
  d_ms, tau_ms   a first-order-plus-delay fit, measured(t) = g LPF_tau(commanded)(t - d) + c, least squares over
                 d in 0-250 ms (5 ms steps) and tau in 5-200 ms;
  g, r2          its gain and the variance it explains.
The loop is closed (the command answers the measured state), so the fit is read for the delay, not as a plant model;
the tracker's own model has no delay and tau_rate 0.12 s.
"""
import argparse
import csv
import glob
import json
import os
import pickle
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, 'tools'))
from run_logs import node_csvs, drone_of  # noqa: E402

RIG_DEFAULT = '/home/wesley/multi_drone_control/results/rig'
GRID_HZ = 250.0
AIR_DZ = 0.15
SKIP_S = 3.0
LAGS = np.arange(0.0, 0.3001, 0.004)
DS = np.arange(0.0, 0.2501, 0.005)
TAUS = np.array([0.001, 0.0025, 0.005, 0.0075, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20])
AXES = ('roll', 'pitch', 'yaw')


def read_bag(bagdir, cache):
    key = None
    if cache:
        os.makedirs(cache, exist_ok=True)
        key = os.path.join(cache, 'rates__' + bagdir.strip('/').replace('/', '__') + '.pkl')
        if os.path.exists(key):
            return pickle.load(open(key, 'rb'))
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bagdir, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in types if t.endswith('/ELRSCommand') or (t.endswith('motion_capture_state') and 'drone' in t)
            or t in ('/fleet/command', '/fleet/abort')]
    out = {'cmd': {}, 'mocap': {}, 'fleet': []}
    if not want:                                 # an empty filter would read every topic
        return out
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    while r.has_next():
        topic, data, tb = r.read_next()
        if topic not in cls:
            continue
        m = deserialize_message(data, cls[topic])
        tb *= 1e-9
        if topic.endswith('/ELRSCommand'):
            out['cmd'].setdefault(int(topic.split('/')[1][6:]), []).append(
                (tb, float(m.armed), m.channel_0, m.channel_1, m.channel_2, m.channel_3))
        elif topic.endswith('motion_capture_state'):
            p, q = m.pose.position, m.pose.orientation
            st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            out['mocap'].setdefault(int(topic.split('/')[1][6:]), []).append((tb, st, p.z, q.w, q.x, q.y, q.z))
        else:
            out['fleet'].append((tb, topic, getattr(m, 'data', '')))
    out['cmd'] = {k: np.array(v) for k, v in out['cmd'].items()}
    out['mocap'] = {k: np.array(v) for k, v in out['mocap'].items()}
    if key:
        pickle.dump(out, open(key, 'wb'))
    return out


def bf_rate(stick, centre, max_rate, expo):
    """Stick -> rate (deg/s), tracker/velocity_loop.betaflight_rates vectorised."""
    a = np.abs(stick)
    return np.sign(stick) * (centre * a + (max_rate - centre) * (expo * a ** 6 + (1.0 - expo) * a ** 2))


def qmul(a, b):
    w1, x1, y1, z1 = a.T
    w2, x2, y2, z2 = b.T
    return np.stack([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2], 1)


def body_rates(st, q):
    """Body-frame rates (deg/s) at samples 1..n-2: the rotation from q[k-1] to q[k+1] over their stamps."""
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    a, b = q[:-2], q[2:].copy()
    b[np.sum(a * b, 1) < 0] *= -1.0
    inv = a * np.array([1.0, -1.0, -1.0, -1.0])
    d = qmul(inv, b)
    v = d[:, 1:]
    s = np.linalg.norm(v, axis=1)
    ang = 2.0 * np.arctan2(s, d[:, 0])
    rv = np.where(s[:, None] > 1e-12, v * (ang / np.maximum(s, 1e-12))[:, None], 0.0)
    dt = st[2:] - st[:-2]
    ok = dt > 1e-4
    return ok, np.degrees(rv / np.where(ok, dt, 1.0)[:, None])


def lpf(x, tau, dt):
    k = dt / (tau + dt)
    y = np.empty_like(x)
    acc = x[0]
    for i, v in enumerate(x):
        acc += k * (v - acc)
        y[i] = acc
    return y


def fit_axis(cmd, meas, dt):
    """(xcorr lag s, d s, tau s, g, r2) for one axis on the common grid."""
    c, m = cmd - cmd.mean(), meas - meas.mean()
    n = len(c)
    best = (-np.inf, 0.0)
    for L in LAGS:
        k = int(round(L / dt))
        if k >= n - 10:
            break
        r = np.corrcoef(c[:n - k], m[k:])[0, 1]
        if r > best[0]:
            best = (r, L)
    fit = (np.inf, 0.0, 0.0, 0.0, 0.0)
    var = float(np.var(m))
    for tau in TAUS:
        y = lpf(c, tau, dt)
        for d in DS:
            k = int(round(d / dt))
            yy, mm = y[:n - k], m[k:]
            A = np.vstack([yy, np.ones_like(yy)]).T
            sol, res, *_ = np.linalg.lstsq(A, mm, rcond=None)
            rss = float(res[0]) if len(res) else float(np.sum((A @ sol - mm) ** 2))
            if rss / len(mm) < fit[0]:
                fit = (rss / len(mm), d, tau, float(sol[0]), 1.0 - (rss / len(mm)) / var)
    return best[1], fit[1], fit[2], fit[3], fit[4], best[0]


def flight_windows(data):
    """(TAKEOFF, end) on the bag clock: the first TAKEOFF to the first LAND, abort or DISARM after it."""
    ev = sorted(data['fleet'])
    to = [t for t, tp, s in ev if tp == '/fleet/command' and s == 'TAKEOFF']
    if not to:
        return None
    end = [t for t, tp, s in ev if t > to[0] and ((tp == '/fleet/command' and s in ('LAND', 'DISARM', 'ESTOP'))
                                                    or tp == '/fleet/abort')]
    return to[0], (end[0] if end else np.inf)


def analyse(date, run, rig, cache):
    bag = os.path.join(rig, date, run)
    data = read_bag(bag, cache)
    w = flight_windows(data)
    rows = []
    if w is None:
        return rows
    prm = {}
    for f in node_csvs(os.path.join(rig, date, run + '_logs'), 'tracker'):
        p = os.path.join(os.path.dirname(f), 'params.json')
        if os.path.exists(p):
            prm[drone_of(f)] = json.load(open(p))
    dt = 1.0 / GRID_HZ
    for d, C in sorted(data['cmd'].items()):
        M = data['mocap'].get(d)
        if M is None or len(M) < 100:
            continue
        pr = prm.get(d, {})
        rates = (float(pr.get('rates_centre_deg', 100.0)), float(pr.get('rates_max_deg', 100.0)),
                 float(pr.get('rates_expo', 0.5)))
        rest = float(np.median(M[M[:, 0] < M[0, 0] + 1.0, 2]))
        ok, wb = body_rates(M[:, 1], M[:, 3:7])
        tm = M[1:-1, 0][ok]
        wb = wb[ok]
        zm = M[1:-1, 2][ok]
        armed = C[C[:, 1] > 0.5]
        if len(armed) < 50:
            continue
        t0, t1 = max(w[0] + SKIP_S, armed[0, 0]), min(w[1], armed[-1, 0])
        grid = np.arange(t0, t1, dt)
        if len(grid) < GRID_HZ * 5:
            continue
        air = np.interp(grid, tm, zm) > rest + AIR_DZ
        grid = grid[air]
        if len(grid) < GRID_HZ * 5:
            continue
        idx = np.clip(np.searchsorted(armed[:, 0], grid, side='right') - 1, 0, None)
        sticks = {'roll': armed[idx, 2], 'pitch': armed[idx, 3], 'yaw': -armed[idx, 5]}
        meas = {'roll': np.interp(grid, tm, wb[:, 0]), 'pitch': np.interp(grid, tm, wb[:, 1]),
                'yaw': np.interp(grid, tm, wb[:, 2])}
        # the grid has holes where the drone was not airborne; the fit treats it as one series
        for ax in AXES:
            cmd = bf_rate(sticks[ax], *rates)
            lag, dd, tau, g, r2, rc = fit_axis(cmd, meas[ax], dt)
            rows.append(dict(date=date, run=run, drone=d, axis=ax, seconds=round(len(grid) * dt, 1),
                             cmd_sd_dps=round(float(np.std(cmd)), 1), meas_sd_dps=round(float(np.std(meas[ax])), 1),
                             xcorr_lag_ms=round(1000 * lag), xcorr_r=round(rc, 2), d_ms=round(1000 * dd),
                             tau_ms=round(1000 * tau), g=round(g, 2), r2=round(r2, 2)))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--dates', nargs='+', default=['2026-10-07', '2026-10-08'])
    ap.add_argument('--rig', default=RIG_DEFAULT)
    ap.add_argument('--out', default=os.path.join(REPO, 'results', 'analysis', 'p6_2026-10-12', '4_latency'))
    ap.add_argument('--cache', default=None)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for date in a.dates:
        for bag in sorted(glob.glob(os.path.join(a.rig, date, '*', '*.db3'))):
            run = os.path.basename(os.path.dirname(bag))
            r = analyse(date, run, a.rig, a.cache)
            print(f'{date}/{run}: {len(r)} drone-axes', file=sys.stderr)
            rows += r
    with open(os.path.join(a.out, 'latency.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('| ' + ' | '.join(rows[0]) + ' |')
    print('|' + '---|' * len(rows[0]))
    for r in rows:
        print('| ' + ' | '.join(str(v) for v in r.values()) + ' |')
    print()
    summary = []
    for date in ['all'] + list(a.dates):
        for ax in AXES:
            s = [r for r in rows if r['axis'] == ax and r['r2'] > 0.2 and date in ('all', r['date'])]
            if not s:
                continue
            q = lambda k: np.percentile([r[k] for r in s], [25, 50, 75])  # noqa: E731
            x, dd, tt = q('xcorr_lag_ms'), q('d_ms'), q('tau_ms')
            summary.append(dict(date=date, axis=ax, drone_flights=len(s), flights=len({r['run'] for r in s}),
                                xcorr_lag_ms=f'{x[1]:.0f} ({x[0]:.0f}-{x[2]:.0f})',
                                d_ms=f'{dd[1]:.0f} ({dd[0]:.0f}-{dd[2]:.0f})',
                                tau_ms=f'{tt[1]:.1f} ({tt[0]:.1f}-{tt[2]:.1f})',
                                g=f"{np.median([r['g'] for r in s]):.2f}", r2=f"{np.median([r['r2'] for r in s]):.2f}"))
    with open(os.path.join(a.out, 'latency_summary.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(summary[0]))
        w.writeheader()
        w.writerows(summary)
    print('Median (IQR) over the drone-flights with r2 > 0.2:\n')
    print('| ' + ' | '.join(summary[0]) + ' |')
    print('|' + '---|' * len(summary[0]))
    for r in summary:
        print('| ' + ' | '.join(str(v) for v in r.values()) + ' |')


if __name__ == '__main__':
    main()
