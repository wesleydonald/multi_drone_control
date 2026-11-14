#!/usr/bin/env python3
"""Q013 "Also": the time series of a whole re-attach flight for the figures, in the columns of the 12 Nov
*_poses.csv (results/analysis/cpp_realtime_2026-11-13/extract.py: t, phase, d0..d3 position and velocity, the ring's
position, quaternion and velocity, the magnet tip, the object, his reference), the whole flight (extract.py stops
3 s after RING_ATTACHED; this does not), with columns appended:
- t_epoch, t_from_takeoff (s after our fleet TAKEOFF line);
- n_ours: the quadrotors our planner flies on the ring (the last FLEET RESIZED line);
- d3_mux: who drives QUAD4's radio (/drone_3/mux_state: partner = his stream, ours = our tracker);
- Tplan_d0..d3: our planner's node-0 planned cable tension per drone (planner log t<slot> via slot2drone, 10 Hz,
  held for at most 0.25 s; empty for a drone not on the ring);
- Tpull_d0..d3: the tracker's estimated cable pull, |acm| x 0.55 kg (its raw measured cable acceleration: the
  mocap acceleration minus the thrust map's thrust and gravity), the mean of the tracker samples in each 50 ms bin;
- event: the log events at their sample (fleet commands, his phase changes, CONTACT, PROOF, ATTACH proven, FLEET
  RESIZED, UNLOAD, DETACH, PARTNER RELEASE).

    source ~/ros2_humble/install/setup.bash && source ~/multi_drone_control/install/setup.bash
    cd ~/multi_drone_control && nice -n 10 python3 results/analysis/rig_results_2026-11-13/reattach_poses.py \
        2026-11-11/nrtfc_2 2026-11-11/nrtfc_5        # -> nrtfc_2_poses.csv, nrtfc_5_poses.csv beside this file

Sampling as extract.py: 20 Hz on the bag clock, each topic's latest message (sample and hold, no age limit), rows
from the first instant every drone's and the ring's mocap has arrived to 10 s after the fleet's "Landed -
disarming" line (the bag runs on, nrtfc_5 for ~7 min); t = s after the first message read. The tip
is /tejen/magnet_tip_pose (as extract.py; /magnet_tip_pose if absent); the object is /pickup/motion_capture_state
(no object flies in nrtfc: the body, if streamed, lies where it was put).
"""
import glob
import os
import re
import sys

import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = os.path.dirname(os.path.abspath(__file__))
RIG = os.path.expanduser('~/multi_drone_control/results/rig')
DRONE_KG = 0.55
MOCAP = ['/drone_0/motion_capture_state', '/drone_1/motion_capture_state', '/drone_2/motion_capture_state',
         '/drone_3/motion_capture_state', '/payload/motion_capture_state', '/pickup/motion_capture_state']
STAMP = re.compile(r'\[(\d{9,}\.\d+)\]')
EVENTS = [
    (re.compile(r"\[fleet_manager\]: Fleet command received: '(\w+)'"), lambda m: f'fleet {m.group(1)}'),
    (re.compile(r'\[fleet_manager\]: Landed - disarming'), lambda m: 'landed'),
    (re.compile(r'\[online_join_planner\]: Planner phase: (\w+) -> (\w+)\.'), lambda m: f'his {m.group(2)}'),
    (re.compile(r'CONTACT \(try (\d+)\): xy ([\d.]+) mm'), lambda m: f'CONTACT try {m.group(1)} ({m.group(2)} mm)'),
    (re.compile(r'PROOF FAILED \(try (\d+), (\w+)\)'), lambda m: f'PROOF FAILED try {m.group(1)} ({m.group(2)})'),
    (re.compile(r'ATTACH: proven \(try (\d+)\)'), lambda m: f'ATTACH proven (try {m.group(1)})'),
    (re.compile(r'FLEET RESIZED to n=(\d+)'), lambda m: f'FLEET RESIZED n={m.group(1)}'),
    (re.compile(r'\[dissipative\] UNLOAD drone'), lambda m: 'UNLOAD (soft detach starts)'),
    (re.compile(r'\[dissipative\] UNLOADED drone'), lambda m: 'UNLOADED'),
    (re.compile(r'\[dissipative\] DETACH drone'), lambda m: 'DETACH'),
    (re.compile(r'\[dissipative\] PARTNER RELEASE'), lambda m: 'PARTNER RELEASE'),
    (re.compile(r'\[dissipative\] ATTACH drone 3 \(OCP resize'), lambda m: 'ATTACH drone 3 (OCP resize)'),
    (re.compile(r'\[dissipative\] reconfiguration hold over'), lambda m: 'path resumes'),
]


def log_events(day, name):
    ev, resize = [], []
    with open(os.path.join(RIG, day, name + '.log'), errors='replace') as f:
        for ln in f:
            s = STAMP.search(ln)
            if not s:
                continue
            for rx, lab in EVENTS:
                m = rx.search(ln)
                if m:
                    ev.append((float(s.group(1)), lab(m)))
                    if lab(m).startswith('FLEET RESIZED'):
                        resize.append((float(s.group(1)), int(m.group(1))))
                    break
    return sorted(ev), resize


def planner(day, name):
    hits = sorted(glob.glob(os.path.join(RIG, day, name + '_logs', 'logs', 'dissipative_planner', '*', 'log.csv')))
    d = pd.read_csv(hits[-1], low_memory=False).sort_values('sim_time').reset_index(drop=True)
    t = d.sim_time.to_numpy(float)
    T = np.full((len(d), 4), np.nan)
    for k in range(len(d)):
        s2d = [int(x) for x in str(d.slot2drone.iloc[k]).split()]
        for s, dr in enumerate(s2d):
            if dr < 4:
                T[k, dr] = d[f't{s}'].iloc[k]
    return t, T


def trackers(day, name):
    out = {}
    for i in range(4):
        hits = sorted(glob.glob(os.path.join(RIG, day, name + '_logs', 'logs', 'tracker', f'drone{i}_*', 'log.csv')))
        if hits:
            d = pd.read_csv(hits[-1], usecols=['sim_time', 'acm_x', 'acm_y', 'acm_z']).sort_values('sim_time')
            out[i] = (d.sim_time.to_numpy(float),
                      DRONE_KG * np.sqrt(d.acm_x.to_numpy(float) ** 2 + d.acm_y.to_numpy(float) ** 2 +
                                         d.acm_z.to_numpy(float) ** 2))
    return out


def extract(fid):
    day, name = fid.split('/')
    bag = os.path.join(RIG, day, name)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    tip_topic = '/tejen/magnet_tip_pose' if '/tejen/magnet_tip_pose' in types else '/magnet_tip_pose'
    other = ['/join_planner/phase', tip_topic, '/join_planner/reference', '/drone_3/mux_state']
    want = [t for t in MOCAP + other if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    last, phase, mux = {}, '', ''
    t0 = next_sample = None
    samples = []
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, cls[topic])
        if t0 is None:
            t0 = next_sample = t
        if topic == '/join_planner/phase':
            phase = m.data
        elif topic == '/drone_3/mux_state':
            mux = m.data
        elif topic == tip_topic:
            p = m.pose.position
            last['tip'] = (p.x, p.y, p.z)
        elif topic == '/join_planner/reference':
            if m.points and m.points[0].transforms:
                p = m.points[0].transforms[0].translation
                last['ref'] = (p.x, p.y, p.z)
        else:
            p, v, q = m.pose.position, m.twist.linear, m.pose.orientation
            last[topic] = (p.x, p.y, p.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z)
        if t >= next_sample and all(k in last for k in MOCAP[:5]):
            next_sample = t + 50_000_000
            row = {'t': (t - t0) / 1e9, 'phase': phase, 't_epoch': t / 1e9, 'd3_mux': mux}
            for i in range(4):
                for j, k in enumerate(('x', 'y', 'z', 'vx', 'vy', 'vz')):
                    row[f'd{i}_{k}'] = last[MOCAP[i]][j]
            rp = last[MOCAP[4]]
            for j, k in enumerate(('x', 'y', 'z')):
                row[f'r_{k}'] = rp[j]
            for j, k in enumerate(('qw', 'qx', 'qy', 'qz')):
                row[f'r_{k}'] = rp[6 + j]
            for j, k in enumerate(('vx', 'vy', 'vz')):
                row[f'r_{k}'] = rp[3 + j]
            for key, pre in (('tip', 'tip'), (MOCAP[5], 'o'), ('ref', 'ref')):
                for j, k in enumerate(('x', 'y', 'z')):
                    row[f'{pre}_{k}'] = last[key][j] if key in last else np.nan
            samples.append(row)
    df = pd.DataFrame(samples)
    ev, resize = log_events(day, name)
    landed = next((t for t, e in ev if e == 'landed'), None)
    if landed is not None:      # the bag runs on after the landing (nrtfc_5: ~7 min): keep 10 s of it
        df = df[df.t_epoch <= landed + 10.0].reset_index(drop=True)
    te = df.t_epoch.to_numpy(float)
    to = next((t for t, e in ev if e == 'fleet TAKEOFF'), None)
    df['t_from_takeoff'] = te - to if to else np.nan
    n = np.full(len(df), np.nan)
    for t, k in resize:
        n[te >= t] = k
    df['n_ours'] = n
    pt, PT = planner(day, name)
    k = np.searchsorted(pt, te, side='right') - 1
    ok = (k >= 0) & (te - pt[np.clip(k, 0, None)] <= 0.25)
    for d in range(4):
        df[f'Tplan_d{d}'] = np.where(ok, PT[np.clip(k, 0, None), d], np.nan)
    TR = trackers(day, name)
    edges = np.r_[te, te[-1] + 0.05]
    for d in range(4):
        col = np.full(len(df), np.nan)
        if d in TR:
            tt, v = TR[d]
            b = np.searchsorted(edges, tt, side='right') - 1
            okb = (b >= 0) & (b < len(df)) & np.isfinite(v)
            s = np.bincount(b[okb], weights=v[okb], minlength=len(df))[:len(df)]
            c = np.bincount(b[okb], minlength=len(df))[:len(df)]
            col = np.where(c > 0, s / np.maximum(c, 1), np.nan)
        df[f'Tpull_d{d}'] = col
    lab = [''] * len(df)
    for t, e in ev:
        i = int(np.searchsorted(te, t, side='left'))
        if 0 <= i < len(df):
            lab[i] = (lab[i] + '; ' + e) if lab[i] else e
    df['event'] = lab
    cols = ['t', 'phase']
    for i in range(4):
        cols += [f'd{i}_{k}' for k in ('x', 'y', 'z', 'vx', 'vy', 'vz')]
    cols += ['r_x', 'r_y', 'r_z', 'r_qw', 'r_qx', 'r_qy', 'r_qz', 'r_vx', 'r_vy', 'r_vz', 'tip_x', 'tip_y', 'tip_z',
             'o_x', 'o_y', 'o_z', 'ref_x', 'ref_y', 'ref_z']
    cols += ['t_epoch', 't_from_takeoff', 'n_ours', 'd3_mux'] + [f'Tplan_d{d}' for d in range(4)] + \
        [f'Tpull_d{d}' for d in range(4)] + ['event']
    fmt = {c: '%.4f' for c in cols if c not in ('phase', 'event', 'd3_mux')}
    out = os.path.join(HERE, f'{name}_poses.csv')
    with open(out, 'w') as f:
        f.write(','.join(cols) + '\n')
        for _, r in df[cols].iterrows():
            vals = []
            for c in cols:
                v = r[c]
                if c in ('phase', 'event', 'd3_mux'):
                    s = '' if (v is None or (isinstance(v, float) and np.isnan(v))) else str(v)
                    vals.append(f'"{s}"' if (',' in s or ';' in s) else s)
                elif c == 't':
                    vals.append(f'{v:.3f}')
                elif c == 't_epoch':
                    vals.append(f'{v:.3f}')
                elif c.startswith('r_q'):
                    vals.append('' if np.isnan(v) else f'{v:.5f}')
                elif c == 'n_ours':
                    vals.append('' if np.isnan(v) else f'{int(v)}')
                else:
                    vals.append('' if np.isnan(v) else f'{v:.4f}' if not c.startswith('T') else f'{v:.3f}')
            f.write(','.join(vals) + '\n')
    print(f'{out}: {len(df)} rows, {df.t.iloc[0]:.2f}-{df.t.iloc[-1]:.2f} s, takeoff at t={to - t0 / 1e9:.2f} s'
          if to else f'{out}: {len(df)} rows')


if __name__ == '__main__':
    for fid in sys.argv[1:] or ['2026-11-11/nrtfc_2', '2026-11-11/nrtfc_5']:
        extract(fid)
