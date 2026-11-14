#!/usr/bin/env python3
"""Q010: the ring's tilt while the returning quadrotor is away, one definition in the real flight and in simulation.

Tilt = the angle between the ring's body z axis and world z, from the payload pose quaternion:
    tilt = acos(1 - 2 (qx^2 + qy^2)).
Window = the magnet off (the ring released) to the re-attachment (the planner's FLEET RESIZED to n=4).

Real flight RIG-1105-sq8_2: every /payload/motion_capture_state sample in the bag (~94 Hz, bag receive time, the laptop's
clock, as the .log's stamps); event stamps from results/rig/2026-11-05/sq8_2.log. Cross-check: the converted run.csv
(50 Hz sample-and-hold grid, the thesis's source).
Simulation R3708, R3712, R3714, R3715: logs/run.csv quaternion columns (sim time, 50 Hz, the twin's
/payload/motion_capture_state), events from logs/events.csv. Those folders moved to the lab PC on 13 Nov
(drones:~/mdc_archive/results/2026-11-05/); copy logs/run.csv and logs/events.csv and pass --sim-root.

usage: q010_tilt.py [--sim-root DIR] [--out DIR]
"""
import argparse
import csv
import glob
import math
import os
import re

import numpy as np
import pandas as pd

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
RIG = os.path.join(REPO, 'results', 'rig', '2026-11-05')
SIMS = ('R3708', 'R3712', 'R3714', 'R3715')


def tilt(qw, qx, qy, qz):
    n = np.sqrt(np.asarray(qw) ** 2 + np.asarray(qx) ** 2 + np.asarray(qy) ** 2 + np.asarray(qz) ** 2)
    zz = 1.0 - 2.0 * ((np.asarray(qx) / n) ** 2 + (np.asarray(qy) / n) ** 2)
    return np.degrees(np.arccos(np.clip(zz, -1.0, 1.0)))


def rotate(q, v):
    """Rotate vector(s) v by unit quaternion(s) q = (w, x, y, z), rows."""
    w, x, y, z = (np.asarray(c, float) for c in q)
    u = np.stack([x, y, z], -1)
    v = np.broadcast_to(np.asarray(v, float), u.shape)
    t = 2.0 * np.cross(u, v)
    return v + w[..., None] * t + np.cross(u, t)


def tilt_from_rest(q, q_rest):
    """Tilt with the body z axis taken as the one that pointed up while the ring lay on the floor (removes a fixed
    offset between the motion-capture body frame and the ring's plane, if the floor is level)."""
    w0, x0, y0, z0 = q_rest
    up_body = rotate((np.array([w0]), np.array([-x0]), np.array([-y0]), np.array([-z0])), [0.0, 0.0, 1.0])[0]
    z = rotate(q, up_body)
    return np.degrees(np.arccos(np.clip(z[:, 2] / np.linalg.norm(z, axis=1), -1.0, 1.0)))


def mean_quat(qw, qx, qy, qz):
    q = np.stack([qw, qx, qy, qz], -1)
    q = q * np.sign(q[:, :1] + 1e-12)
    m = q.mean(0)
    return m / np.linalg.norm(m)


def stats(t, v, a, b):
    """Sample mean, median, time-weighted mean (sample-and-hold), min, max, and the count over [a, b]."""
    m = (t >= a) & (t <= b)
    tt, vv = t[m], v[m]
    if len(tt) < 2:
        return dict(mean=math.nan, median=math.nan, tmean=math.nan, min=math.nan, max=math.nan, n=len(tt))
    dt = np.diff(np.append(tt, b))
    return dict(mean=float(vv.mean()), median=float(np.median(vv)), tmean=float((vv * dt).sum() / dt.sum()),
                min=float(vv.min()), max=float(vv.max()), n=int(len(tt)))


def at(t, v, x):
    i = np.searchsorted(t, x, side='right') - 1
    return float(v[max(i, 0)])


def rig_events():
    log = open(os.path.join(RIG, 'sq8_2.log')).read().splitlines()

    def stamp(pat, k=0):
        hits = [l for l in log if re.search(pat, l)]
        return float(re.search(r'\[(\d{10}\.\d+)\]', hits[k]).group(1))

    return {'unload': stamp(r'UNLOAD drone 4'),
            'resize_n3': stamp(r'FLEET RESIZED to n=3'),
            'magnet_off': stamp(r'magnet_attachment_manager\]: Magnet command: OFF'),
            'partner_release': stamp(r'PARTNER RELEASE'),
            'contact': stamp(r'CONTACT \(try 1\)'),
            'proven': stamp(r'ATTACH: proven'),
            'resize_n4': stamp(r'FLEET RESIZED to n=4', 1),       # the first is the start weld at launch
            'land': stamp(r"Fleet command received: 'LAND'"),
            'takeoff': stamp(r"Fleet command received: 'TAKEOFF'")}


def rig_bag():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=os.path.join(RIG, 'sq8_2'), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {x.name: x.type for x in r.get_all_topics_and_types()}
    topic = '/payload/motion_capture_state'
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    cls = get_message(types[topic])
    rows = []
    while r.has_next():
        _, data, ts = r.read_next()
        m = deserialize_message(data, cls)
        o = m.pose.orientation
        rows.append((ts * 1e-9, m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, o.w, o.x, o.y, o.z))
    d = pd.DataFrame(rows, columns=['t', 'stamp', 'qw', 'qx', 'qy', 'qz']).sort_values('t')
    d['tilt'] = tilt(d.qw, d.qx, d.qy, d.qz)
    return d


def rig_runcsv():
    import json
    base = os.path.join(RIG, 'sq8_2_run')
    t0 = json.load(open(os.path.join(base, 'manifest.json')))['t0_epoch']
    r = pd.read_csv(os.path.join(base, 'logs', 'run.csv'), usecols=['t', 'payload_qw', 'payload_qx', 'payload_qy',
                                                                     'payload_qz', 'payload_tilt_deg'])
    r['t'] = r.t + t0
    return r


def sim_run(root, rid):
    (d,) = glob.glob(os.path.join(root, f'{rid}_*'))
    r = pd.read_csv(os.path.join(d, 'logs', 'run.csv'), usecols=['t', 'payload_qw', 'payload_qx', 'payload_qy',
                                                                  'payload_qz', 'payload_tilt_deg'])
    r = r.drop_duplicates('t').sort_values('t')
    r['tilt'] = tilt(r.payload_qw, r.payload_qx, r.payload_qy, r.payload_qz)
    ev = pd.read_csv(os.path.join(d, 'logs', 'events.csv'))
    ev = ev[pd.to_numeric(ev.sim_time, errors='coerce').notna()]
    g = lambda k: float(ev.loc[ev.event == k, 'sim_time'].astype(float).iloc[0])  # noqa: E731
    e = {'takeoff': g('TAKEOFF'), 'detach_cmd': g('DETACH'), 'magnet_off': g('DETACH_RESIZE'), 'released': g('RELEASED'),
         'partner_release': g('PARTNER_RELEASE'), 'contact': g('CONTACT'), 'reweld': g('REWELD'),
         'fold_in': float(ev.loc[(ev.event == 'FOLD_IN'), 'sim_time'].astype(float).iloc[-1])}
    return r, e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sim-root', default=os.path.join(REPO, 'results', '2026-11-05'))
    ap.add_argument('--out', default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()
    rows, sens, prof = [], [], []

    # ---- real flight ----
    e = rig_events()
    bag = rig_bag()
    rc = rig_runcsv()
    t, v = bag.t.to_numpy(), bag.tilt.to_numpy()
    tc, vc = rc.t.to_numpy(), rc.payload_tilt_deg.to_numpy()
    t_off, t_n4 = e['magnet_off'], e['resize_n4']
    s = stats(t, v, t_off, t_n4)
    sc = stats(tc, vc, t_off, t_n4)
    rest = stats(t, v, t[0], e['takeoff'])
    rows.append(dict(run='RIG-1105-sq8_2', source='bag /payload/motion_capture_state (all samples)', start='magnet OFF',
                     end='FLEET RESIZED n=4', t_start=round(t_off, 3), t_end=round(t_n4, 3),
                     window_s=round(t_n4 - t_off, 3), **{k: round(x, 2) if isinstance(x, float) else x
                                                         for k, x in s.items()},
                     tilt_at_start=round(at(t, v, t_off), 2), tilt_at_end=round(at(t, v, t_n4), 2)))
    rows.append(dict(run='RIG-1105-sq8_2', source='sq8_2_run/logs/run.csv (50 Hz grid)', start='magnet OFF',
                     end='FLEET RESIZED n=4', t_start=round(t_off, 3), t_end=round(t_n4, 3),
                     window_s=round(t_n4 - t_off, 3), **{k: round(x, 2) if isinstance(x, float) else x
                                                         for k, x in sc.items()},
                     tilt_at_start=round(at(tc, vc, t_off), 2), tilt_at_end=round(at(tc, vc, t_n4), 2)))
    # the window's ends, moved
    alts = [('magnet OFF', t_off, 'FLEET RESIZED n=4', t_n4),
            ('UNLOAD start (3.3 s before the magnet off)', e['unload'], 'FLEET RESIZED n=4', t_n4),
            ('magnet OFF', t_off, 'CONTACT (try 1)', e['contact']),
            ('magnet OFF', t_off, 'ATTACH proven', e['proven']),
            ('partner release', e['partner_release'], 'FLEET RESIZED n=4', t_n4),
            ('magnet OFF + 1 s', t_off + 1, 'FLEET RESIZED n=4', t_n4),
            ('magnet OFF + 2 s', t_off + 2, 'FLEET RESIZED n=4', t_n4),
            ('magnet OFF', t_off, 'FLEET RESIZED n=4 - 1 s', t_n4 - 1),
            ('magnet OFF', t_off, 'FLEET RESIZED n=4 - 2 s', t_n4 - 2),
            ('magnet OFF', t_off, 'FLEET RESIZED n=4 + 1 s', t_n4 + 1),
            ('magnet OFF', t_off, 'FLEET RESIZED n=4 + 2 s', t_n4 + 2),
            ('magnet OFF', t_off, 'magnet OFF + 10 s', t_off + 10),
            ('magnet OFF + 10 s', t_off + 10, 'FLEET RESIZED n=4', t_n4)]
    for sa, ta, sb, tb in alts:
        x = stats(t, v, ta, tb)
        sens.append(dict(run='RIG-1105-sq8_2', start=sa, end=sb, window_s=round(tb - ta, 2), mean=round(x['mean'], 2),
                         tmean=round(x['tmean'], 2), median=round(x['median'], 2), min=round(x['min'], 2),
                         max=round(x['max'], 2)))
    pre = bag[bag.t < e['takeoff']]
    vr = tilt_from_rest((bag.qw, bag.qx, bag.qy, bag.qz), mean_quat(pre.qw, pre.qx, pre.qy, pre.qz))
    x = stats(t, vr, t_off, t_n4)
    sens.append(dict(run='RIG-1105-sq8_2', start='magnet OFF', end='FLEET RESIZED n=4 (tilt from the floor attitude)',
                     window_s=round(t_n4 - t_off, 2), mean=round(x['mean'], 2), tmean=round(x['tmean'], 2),
                     median=round(x['median'], 2), min=round(x['min'], 2), max=round(x['max'], 2)))
    for k in range(0, 23):
        prof.append(dict(run='RIG-1105-sq8_2', t_rel_magnet_off_s=k, tilt_deg=round(at(t, v, t_off + k), 2)))
    rig_times = dict(off_to_n4=t_n4 - t_off, rel_to_n4=t_n4 - e['partner_release'], off_to_rel=e['partner_release'] - t_off,
                     rel_to_contact=e['contact'] - e['partner_release'], rel_to_proven=e['proven'] - e['partner_release'],
                     off_to_contact=e['contact'] - t_off, off_to_proven=e['proven'] - t_off)

    # ---- simulation ----
    sim = {}
    for rid in SIMS:
        r, ev = sim_run(a.sim_root, rid)
        ts, vs = r.t.to_numpy(), r.tilt.to_numpy()
        x = stats(ts, vs, ev['released'], ev['reweld'])
        xo = stats(ts, r.payload_tilt_deg.to_numpy(), ev['released'], ev['reweld'])
        assert abs(x['mean'] - xo['mean']) < 0.01, (rid, x, xo)       # the logged column = the quaternion's tilt
        sim[rid] = (x, ev)
        rows.append(dict(run=rid, source='logs/run.csv quaternion (sim time, 50 Hz)', start='RELEASED',
                         end='REWELD', t_start=round(ev['released'], 3), t_end=round(ev['reweld'], 3),
                         window_s=round(ev['reweld'] - ev['released'], 3),
                         **{k: round(y, 2) if isinstance(y, float) else y for k, y in x.items()},
                         tilt_at_start=round(at(ts, vs, ev['released']), 2), tilt_at_end=round(at(ts, vs, ev['reweld']), 2)))
        for sa, ta, sb, tb in [('magnet OFF (DETACH_RESIZE)', ev['magnet_off'], 'REWELD', ev['reweld']),
                               ('partner release', ev['partner_release'], 'REWELD', ev['reweld']),
                               ('RELEASED', ev['released'], 'CONTACT', ev['contact']),
                               ('RELEASED + 1 s', ev['released'] + 1, 'REWELD', ev['reweld']),
                               ('RELEASED + 2 s', ev['released'] + 2, 'REWELD', ev['reweld']),
                               ('RELEASED', ev['released'], 'REWELD - 2 s', ev['reweld'] - 2),
                               ('RELEASED', ev['released'], 'REWELD + 2 s', ev['reweld'] + 2),
                               ('RELEASED', ev['released'], 'RELEASED + 10 s', ev['released'] + 10),
                               ('RELEASED + 10 s', ev['released'] + 10, 'REWELD', ev['reweld'])]:
            y = stats(ts, vs, ta, tb)
            sens.append(dict(run=rid, start=sa, end=sb, window_s=round(tb - ta, 2), mean=round(y['mean'], 2),
                             tmean=round(y['tmean'], 2), median=round(y['median'], 2), min=round(y['min'], 2),
                             max=round(y['max'], 2)))
        pre = r[r.t < ev['takeoff']]
        vr = tilt_from_rest((r.payload_qw, r.payload_qx, r.payload_qy, r.payload_qz),
                            mean_quat(pre.payload_qw, pre.payload_qx, pre.payload_qy, pre.payload_qz))
        y = stats(ts, vr, ev['released'], ev['reweld'])
        sens.append(dict(run=rid, start='RELEASED', end='REWELD (tilt from the floor attitude)',
                         window_s=round(ev['reweld'] - ev['released'], 2), mean=round(y['mean'], 2),
                         tmean=round(y['tmean'], 2), median=round(y['median'], 2), min=round(y['min'], 2),
                         max=round(y['max'], 2)))
        for k in range(0, 21):
            prof.append(dict(run=rid, t_rel_magnet_off_s=k, tilt_deg=round(at(ts, vs, ev['released'] + k), 2)))

    os.makedirs(a.out, exist_ok=True)
    for name, data in (('tilt_while_away.csv', rows), ('window_sensitivity.csv', sens), ('tilt_profile_1s.csv', prof)):
        with open(os.path.join(a.out, name), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(data[0].keys()), lineterminator='\n')
            w.writeheader()
            w.writerows(data)

    print('RIG-1105-sq8_2 events (epoch):', {k: round(x, 3) for k, x in e.items()})
    print('  times (s):', {k: round(x, 2) for k, x in rig_times.items()})
    print(f"  tilt at rest before takeoff (bag): mean {rest['mean']:.2f} deg (body-frame offset bound)")
    print(f"  bag    : mean {s['mean']:.2f}  tmean {s['tmean']:.2f}  median {s['median']:.2f}  n {s['n']}  "
          f"start {at(t, v, t_off):.2f}  end {at(t, v, t_n4):.2f}  min {s['min']:.2f} max {s['max']:.2f}")
    print(f"  run.csv: mean {sc['mean']:.2f}  median {sc['median']:.2f}  n {sc['n']}")
    for rid in SIMS:
        x, ev = sim[rid]
        print(f"{rid}: RELEASED {ev['released']:.3f} REWELD {ev['reweld']:.3f} ({ev['reweld'] - ev['released']:.2f} s; "
              f"from the partner release {ev['reweld'] - ev['partner_release']:.2f} s)  mean {x['mean']:.2f}  "
              f"median {x['median']:.2f}  min {x['min']:.2f} max {x['max']:.2f}")
    m4 = np.mean([sim[r][0]['mean'] for r in SIMS])
    print(f'mean of the four simulation means: {m4:.3f}')
    for x in sens:
        print('  sens', x)


if __name__ == '__main__':
    main()
