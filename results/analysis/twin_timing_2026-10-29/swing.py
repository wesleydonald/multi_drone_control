#!/usr/bin/env python3
"""QUAD4's swing and magnet-to-target error per phase window, the same way for a rig bag and a twin run.

    swing.py rig  <bag dir> [--mpc <his log.csv>] [--plate DX DY]
    swing.py twin <run dir> [--mpc <his log.csv>] [--plate DX DY] [--json out.json]

Swing (cable_geometry.py's math): anchor = body + R(q)(0, 0, -0.030) (twin: body - 0.030 z, run.csv has no attitude;
< 0.1 deg at QUAD4's tilts), r = magnet - anchor, phi = atan2(r_x, -r_z), theta = atan2(r_y, -r_z), sampled and held at
30 Hz. Per window of one phase (>= 1 s): each angle's mean and its oscillation (std about the window mean); the
total swing rms = sqrt(std_phi^2 + std_theta^2); the dominant frequency of the larger axis (Welch).
Target: plate 9 = ring pose (x, y, q) + R(q)(DX, DY, 0) (rig: the typed attach_x/y_offset -0.005 / 0.197; twin 0 / 0.204);
error = |magnet_xy - target_xy|.
"""
import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
from scipy.signal import welch

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

ANCHOR_Z = -0.030
PHASES = ('APPROACH_ABOVE_TARGET', 'MATCH_VELOCITY', 'DESCEND_TO_ATTACHMENT', 'SETTLE_ABOVE_PICKUP')
MD = ('MATCH_VELOCITY', 'DESCEND_TO_ATTACHMENT')


def rotm(q):
    w, x, y, z = (q / np.linalg.norm(q, axis=1, keepdims=True)).T
    return np.stack([np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
                     np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
                     np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def hold(a, tq):
    return a[np.clip(np.searchsorted(a[:, 0], tq, side='right') - 1, 0, None)]


def dominant(x, fs=30.0):
    if len(x) < 16:
        return float('nan')
    f, P = welch(x - x.mean(), fs=fs, nperseg=min(256, len(x)))
    m = f > 0.1
    return float(f[m][np.argmax(P[m])])


def read_rig(bag):
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id='sqlite3'), ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    body, tip, ring, phases = [], [], [], []
    want = ('/drone_3/motion_capture_state', '/magnet_tip_pose', '/payload/motion_capture_state', '/join_planner/phase')
    while r.has_next():
        topic, raw, t = r.read_next()
        if topic not in want:
            continue
        m = deserialize_message(raw, get_message(types[topic]))
        t = t * 1e-9
        if topic == '/join_planner/phase':
            if not phases or phases[-1][1] != m.data:
                phases.append((t, m.data))
            continue
        p = m.pose.position
        if topic == '/magnet_tip_pose':
            tip.append([t, p.x, p.y, p.z])
        else:
            q = m.pose.orientation
            (body if topic.startswith('/drone_3') else ring).append([t, p.x, p.y, p.z, q.w, q.x, q.y, q.z])
    return np.array(body), np.array(tip), np.array(ring), phases


def read_twin(run):
    logs = os.path.join(run, 'logs')
    d = pd.read_csv(os.path.join(logs, 'run.csv'))
    ev = pd.read_csv(os.path.join(logs, 'events.csv'))
    phases = [(float(r.sim_time), r.event.split(':', 1)[1]) for r in ev.itertuples()
              if isinstance(r.event, str) and r.event.startswith('PHASE:')]
    t = d.t.values
    ok = d[['d3_x', 'tip_x', 'payload_x']].notna().all(axis=1).values
    body = np.c_[t, d.d3_x, d.d3_y, d.d3_z, np.ones(len(d)), np.zeros((len(d), 3))][ok]
    tip = np.c_[t, d.tip_x, d.tip_y, d.tip_z][ok]
    ring = np.c_[t, d.payload_x, d.payload_y, d.payload_z, d.payload_qw, d.payload_qx, d.payload_qy, d.payload_qz][ok]
    tgt = np.c_[t, d.tip_target_x, d.tip_target_y, d.tip_target_z][ok]
    wall = np.c_[d.wall.values, t]
    return body, tip, ring, phases, tgt, wall, ev


def windows(phases, t_end):
    out = []
    for i, (t0, ph) in enumerate(phases):
        t1 = phases[i + 1][0] if i + 1 < len(phases) else t_end
        out.append((t0, t1, ph))
    return out


def analyse(body, tip, ring, phases, plate, mpc=None, mpc_time=None, tgt=None, t_end=None):
    t_end = t_end if t_end is not None else min(body[-1, 0], tip[-1, 0])
    res = {'windows': []}
    md_err, md_err_logged = [], []
    for t0, t1, ph in windows(phases, t_end):
        if ph not in PHASES or t1 <= t0:
            continue
        tq = np.arange(t0, t1, 1 / 30)
        if len(tq) < 2:
            continue
        b, m, g = hold(body, tq), hold(tip, tq), hold(ring, tq)
        R = rotm(b[:, 4:8])
        r = m[:, 1:4] - (b[:, 1:4] + R @ np.array([0.0, 0.0, ANCHOR_Z]))
        phi = np.degrees(np.arctan2(r[:, 0], -r[:, 2]))
        th = np.degrees(np.arctan2(r[:, 1], -r[:, 2]))
        target = g[:, 1:3] + (rotm(g[:, 4:8]) @ np.array([plate[0], plate[1], 0.0]))[:, :2]
        err = np.linalg.norm(m[:, 1:3] - target, axis=1)
        w = {'phase': ph, 't0': round(t0, 3), 'dur': round(t1 - t0, 3), 'err_med_cm': float(np.median(err) * 100),
             'err_max_cm': float(err.max() * 100), 'chord_m': float(np.median(np.linalg.norm(r, axis=1)))}
        if tgt is not None:
            e2 = np.linalg.norm(m[:, 1:3] - hold(tgt, tq)[:, 1:3], axis=1)
            w['err_logged_med_cm'] = float(np.median(e2) * 100)
            if ph in MD:
                md_err_logged.append(e2)
        if ph in MD:
            md_err.append(err)
        if t1 - t0 >= 1.0:
            sp, st = float(phi.std()), float(th.std())
            w.update({'phi_mean': float(phi.mean()), 'theta_mean': float(th.mean()), 'phi_rms': sp, 'theta_rms': st,
                      'total_rms': float(np.hypot(sp, st)), 'lean': float(np.hypot(phi.mean(), th.mean())),
                      'f_dom': dominant(phi if sp >= st else th)})
            if mpc is not None:
                mt = mpc_time(mpc.timestamp.values)
                sel = mpc[(mt >= t0) & (mt < t1)]
                if len(sel) > 2:
                    w['mpc_phi_rms'] = float(np.degrees(sel.phi).std())
                    w['mpc_theta_rms'] = float(np.degrees(sel.theta).std())
        res['windows'].append(w)
    full = [w for w in res['windows'] if 'total_rms' in w]
    md = [w for w in full if w['phase'] in MD]
    res['md_windows'] = len(md)
    res['S_md_median'] = float(np.median([w['total_rms'] for w in md])) if md else float('nan')
    res['S_md_max'] = float(max(w['total_rms'] for w in md)) if md else float('nan')
    res['axis_rms_md'] = [round(max(w['phi_rms'], w['theta_rms']), 2) for w in md]
    ap = [w for w in full if w['phase'] == 'APPROACH_ABOVE_TARGET']
    res['S_approach_max'] = float(max(w['total_rms'] for w in ap)) if ap else float('nan')
    res['lean_median'] = float(np.median([w['lean'] for w in full])) if full else float('nan')
    res['f_dom_md'] = [round(w['f_dom'], 2) for w in md]
    res['E_md_median_cm'] = float(np.median(np.concatenate(md_err)) * 100) if md_err else float('nan')
    if md_err_logged:
        res['E_md_logged_median_cm'] = float(np.median(np.concatenate(md_err_logged)) * 100)
    st = [w for w in full if w['phase'] == 'SETTLE_ABOVE_PICKUP']
    if st:
        res['settle'] = {'total_rms': st[0]['total_rms'], 'err_med_cm': st[0]['err_med_cm'], 'dur': st[0]['dur']}
    names = [p for _, p in phases]
    res['n_match'] = names.count('MATCH_VELOCITY')
    res['n_descend'] = names.count('DESCEND_TO_ATTACHMENT')
    res['match_durs'] = [w['dur'] for w in res['windows'] if w['phase'] == 'MATCH_VELOCITY']
    res['descent_durs'] = [w['dur'] for w in res['windows'] if w['phase'] == 'DESCEND_TO_ATTACHMENT']
    # a descent that ends back in APPROACH is an abort
    res['n_abort'] = sum(1 for i, p in enumerate(names[:-1])
                         if p == 'DESCEND_TO_ATTACHMENT' and names[i + 1] == 'APPROACH_ABOVE_TARGET')
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('kind', choices=('rig', 'twin'))
    ap.add_argument('path')
    ap.add_argument('--mpc')
    ap.add_argument('--plate', nargs=2, type=float)
    ap.add_argument('--json')
    a = ap.parse_args()
    mpc = pd.read_csv(a.mpc) if a.mpc else None
    if a.kind == 'rig':
        body, tip, ring, phases = read_rig(a.path)
        plate = a.plate or (-0.005, 0.197)
        res = analyse(body, tip, ring, phases, plate, mpc, (lambda x: x))
    else:
        body, tip, ring, phases, tgt, wall, ev = read_twin(a.path)
        plate = a.plate or (0.0, 0.204)
        mpc_time = None
        if mpc is not None:
            from align import sim_time_of
            mpc_time, _, resid = sim_time_of(pd.read_csv(os.path.join(a.path, 'logs', 'run.csv')), mpc)
            print(f'his log aligned to sim time (height residual {resid * 1000:.1f} mm)')
        res = analyse(body, tip, ring, phases, plate, mpc, mpc_time, tgt)
        evs = ev.event.astype(str)
        for k in ('CONTACT', 'WELD', 'LAND', 'LIFTED'):
            hit = ev[evs == k]
            res[k.lower()] = float(hit.sim_time.iloc[0]) if len(hit) else None
        tk = ev[evs == 'PHASE:TAKEOFF']
        res['his_takeoff'] = float(tk.sim_time.iloc[0]) if len(tk) else None
        res['phases'] = [(round(t, 2), p) for t, p in phases]
    res['path'] = a.path
    for w in res['windows']:
        extra = (f" phi {w['phi_mean']:+.2f}/{w['phi_rms']:.2f} theta {w['theta_mean']:+.2f}/{w['theta_rms']:.2f} "
                 f"tot {w['total_rms']:.2f} f {w['f_dom']:.2f}" if 'total_rms' in w else '')
        mp = (f" mpc {w['mpc_phi_rms']:.2f}/{w['mpc_theta_rms']:.2f}" if 'mpc_phi_rms' in w else '')
        lg = f" (logged {w['err_logged_med_cm']:.1f})" if 'err_logged_med_cm' in w else ''
        print(f"{w['t0']:8.2f} {w['phase'][:12]:12s} {w['dur']:5.2f} s err {w['err_med_cm']:4.1f}{lg} cm "
              f"chord {w['chord_m']:.3f}{extra}{mp}")
    keys = ('md_windows', 'S_md_median', 'S_md_max', 'axis_rms_md', 'S_approach_max', 'lean_median', 'f_dom_md',
            'E_md_median_cm', 'E_md_logged_median_cm', 'settle', 'n_match', 'n_descend', 'n_abort', 'match_durs',
            'his_takeoff', 'contact', 'weld')
    for k in keys:
        if k in res:
            v = res[k]
            print(f'  {k}: {round(v, 2) if isinstance(v, float) else v}')
    if a.json:
        with open(a.json, 'w') as f:
            json.dump(res, f, indent=1)


if __name__ == '__main__':
    sys.exit(main())
