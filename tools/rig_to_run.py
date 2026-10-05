#!/usr/bin/env python3
"""tools/rig_to_run.py -- turn one rig flight into a run directory that metrics.py / plot_run.py read like a sim run.

    python3 tools/rig_to_run.py --out results/rig/2026-09-30/r3a_f1_run \\
        --trackers results/rig/2026-09-30/r3a_f1_logs/logs/controller_quad_load/planner_drone{0,1,2}_*  \\
        [--bag results/rig/2026-09-30/r3a_f1] [--t2-log results/rig/2026-09-30/r3a_f1.log]

On the rig every node stamps wall-clock epoch time (no use_sim_time), and so do the bag and the tee'd T2 console,
so the three line up directly. The flight is resampled on a 50 Hz grid (last sample at or before each tick) into
logs/run.csv with the shared t / payload_* / dN_* schema (readiness review 2026-09-29: rig flights could not be
scored). Drone pose, velocity and tilt come from the bag's mocap when given, else the tracker logs; the ring's
orientation (payload_tilt_deg) needs the bag. Throttle (dN_thr = u2), references and armed come from the trackers.
Events (ARM, TAKEOFF, LAND, ESTOP, DISARM, FLEET_ABORT, LANDED, and the manager's grounding lines) come from the T2
console. t = 0 at the first grid tick; events.csv uses the same origin."""
import argparse
import glob
import json
import math
import os
import re
import sys

import numpy as np
import pandas as pd

HZ = 50.0
EVENT_RES = [
    (re.compile(r"\[(\d{9,}\.\d+)\] \[(?:fleet_manager|central_controller)\]: Fleet command received: '(\w+)'"), None),
    (re.compile(r"\[(\d{9,}\.\d+)\] \[(?:fleet_manager|central_controller)\]: EMERGENCY STOP"), 'FLEET_ABORT'),
    (re.compile(r"\[(\d{9,}\.\d+)\] \[(?:fleet_manager|central_controller)\]: Landed - disarming"), 'LANDED'),
    (re.compile(r"\[(\d{9,}\.\d+)\] \[(?:fleet_manager|central_controller)\]: (Drone \d disarmed before TAKEOFF)"), 'GROUNDED'),
    (re.compile(r"\[(\d{9,}\.\d+)\] \[(?:fleet_manager|central_controller)\]: (TAKEOFF REFUSED[^\n]*)"), 'GROUNDED'),
    (re.compile(r"\[(\d{9,}\.\d+)\] \[\w+\]: \[planner\] handover settle complete"), 'HANDOVER'),
]


def tilt_deg(qw, qx, qy, qz):
    """Angle between the body z axis and world z (deg)."""
    zz = 1.0 - 2.0 * (np.asarray(qx) ** 2 + np.asarray(qy) ** 2)
    return np.degrees(np.arccos(np.clip(zz, -1.0, 1.0)))


def read_tracker(path):
    f = path if path.endswith('.csv') else os.path.join(path, 'log.csv')
    d = pd.read_csv(f)
    return d.sort_values('sim_time').reset_index(drop=True)


def read_bag(path):
    """{topic: DataFrame(t, x, y, z, qw, qx, qy, qz, vx, vy, vz)} for */motion_capture_state, and ELRS armed."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    mocap, armed = {}, {}
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic.endswith('motion_capture_state'):
            m = deserialize_message(data, get_message(types[topic]))
            p, o, v = m.pose.position, m.pose.orientation, m.twist.linear
            mocap.setdefault(topic, []).append((ts * 1e-9, p.x, p.y, p.z, o.w, o.x, o.y, o.z, v.x, v.y, v.z))
        elif re.fullmatch(r'/drone_\d+/ELRSCommand', topic):
            m = deserialize_message(data, get_message(types[topic]))
            armed.setdefault(topic, []).append((ts * 1e-9, float(bool(m.armed))))
    cols = ['t', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz']
    return ({k: pd.DataFrame(v, columns=cols) for k, v in mocap.items()},
            {k: pd.DataFrame(v, columns=['t', 'armed']) for k, v in armed.items()})


def events_from_log(path):
    out = []
    text = open(path, errors='replace').read()
    for rx, name in EVENT_RES:
        for m in rx.finditer(text):
            ev = name or m.group(2).upper()
            arg = m.group(2) if name == 'GROUNDED' else ''
            out.append((float(m.group(1)), ev, arg))
    return sorted(set(out))


def on_grid(grid, t, values):
    """Last sample at or before each grid tick (NaN before the first sample)."""
    idx = np.searchsorted(np.asarray(t), grid, side='right') - 1
    out = np.asarray(values, float)[np.clip(idx, 0, None)]
    return np.where(idx >= 0, out, np.nan)


def convert(trackers, out, bag=None, t2_log=None):
    tr = {i: read_tracker(p) for i, p in enumerate(trackers)}
    mocap, armed = read_bag(bag) if bag else ({}, {})
    t0 = max(d.sim_time.iloc[0] for d in tr.values())
    t1 = min(d.sim_time.iloc[-1] for d in tr.values())
    if t1 <= t0:
        raise ValueError('the tracker logs do not overlap in time')
    grid = np.arange(t0, t1, 1.0 / HZ)
    col = {'t': grid - t0, 'wall': grid - t0, 'step': np.arange(len(grid))}
    pay = mocap.get('/payload/motion_capture_state')
    d0 = tr[0]
    if pay is not None and len(pay):
        for k, src in (('x', 'x'), ('y', 'y'), ('z', 'z'), ('qw', 'qw'), ('qx', 'qx'), ('qy', 'qy'), ('qz', 'qz')):
            col[f'payload_{k}'] = on_grid(grid, pay.t, pay[src])
        col['payload_tilt_deg'] = tilt_deg(col['payload_qw'], col['payload_qx'], col['payload_qy'], col['payload_qz'])
    else:
        for k in ('x', 'y', 'z'):
            col[f'payload_{k}'] = on_grid(grid, d0.sim_time, d0[f'payload_{k}']) if f'payload_{k}' in d0 else np.nan
        for k in ('qw', 'qx', 'qy', 'qz', 'tilt_deg'):
            col[f'payload_{k}'] = np.full(len(grid), np.nan)
    for k in ('x', 'y', 'z'):
        c = f'payload_ref_{k}'
        col[c] = on_grid(grid, d0.sim_time, d0[c]) if c in d0 else np.full(len(grid), np.nan)
    for i, d in tr.items():
        mc = mocap.get(f'/drone_{i}/motion_capture_state')
        if mc is not None and len(mc):
            src_t, get = mc.t, (lambda k, mc=mc: mc[k])
        else:
            src_t, get = d.sim_time, (lambda k, d=d: d[f'pose_{k}'] if k in ('x', 'y', 'z', 'qw', 'qx', 'qy', 'qz')
                                      else d[f'x0_{k}'] if f'x0_{k}' in d else pd.Series(np.nan, index=d.index))
        for k in ('x', 'y', 'z'):
            col[f'd{i}_{k}'] = on_grid(grid, src_t, get(k))
        for k in ('vx', 'vy', 'vz'):
            col[f'd{i}_{k}'] = on_grid(grid, src_t, get(k))
        q = [on_grid(grid, src_t, get(k)) for k in ('qw', 'qx', 'qy', 'qz')]
        col[f'd{i}_tilt_deg'] = tilt_deg(*q)
        for k in ('x', 'y', 'z'):
            col[f'd{i}_ref_{k}'] = on_grid(grid, d.sim_time, d[f'ref_{k}'])
        col[f'd{i}_track_err'] = np.sqrt(sum((col[f'd{i}_{k}'] - col[f'd{i}_ref_{k}']) ** 2 for k in 'xyz'))
        for k in ('acm', 'tension', 'elev_deg'):
            col[f'd{i}_{k}'] = np.full(len(grid), np.nan)
        col[f'd{i}_thr'] = on_grid(grid, d.sim_time, d['u2'])
        a = armed.get(f'/drone_{i}/ELRSCommand')
        col[f'd{i}_armed'] = on_grid(grid, a.t, a.armed) if a is not None and len(a) else np.full(len(grid), np.nan)
        col[f'd{i}_attached'] = np.full(len(grid), np.nan)
    os.makedirs(os.path.join(out, 'logs'), exist_ok=True)
    pd.DataFrame(col).to_csv(os.path.join(out, 'logs', 'run.csv'), index=False)
    evs = [(t - t0, e, a) for t, e, a in (events_from_log(t2_log) if t2_log else []) if t0 - 5 <= t <= t1 + 5]
    pd.DataFrame(evs, columns=['sim_time', 'event', 'arg']).to_csv(os.path.join(out, 'logs', 'events.csv'), index=False)
    manifest = {'kind': 'rig', 'n_drones': len(tr), 'trackers': [os.path.abspath(p) for p in trackers],
                'bag': os.path.abspath(bag) if bag else None, 't2_log': os.path.abspath(t2_log) if t2_log else None,
                't0_epoch': t0, 'duration_s': t1 - t0, 'grid_hz': HZ,
                'sources': {'drone_pose': 'bag mocap' if mocap else 'tracker log',
                            'payload_orientation': 'bag mocap' if pay is not None else 'none (no bag)'}}
    with open(os.path.join(out, 'manifest.json'), 'w') as fh:
        json.dump(manifest, fh, indent=2)
    return manifest, len(grid), len(evs)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--out', required=True)
    ap.add_argument('--trackers', nargs='+', required=True, help='tracker session dirs (logs/tracker/droneN_*, before 3 Oct planner_droneN_*) or log.csv, drone order')
    ap.add_argument('--bag')
    ap.add_argument('--t2-log')
    a = ap.parse_args(argv)
    trackers = [p for pat in a.trackers for p in sorted(glob.glob(pat))] or a.trackers
    man, n, ne = convert(trackers, a.out, a.bag, a.t2_log)
    print(f'{a.out}: {n} rows ({man["duration_s"]:.1f} s, {man["n_drones"]} drones), {ne} events; '
          f'drone pose from {man["sources"]["drone_pose"]}, ring orientation from {man["sources"]["payload_orientation"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
