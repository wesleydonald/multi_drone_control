#!/usr/bin/env python3
"""Replay a rig bag through the attach confirmation (drone_magnet/comotion_proof.py): every contact, proof and
failure the co-motion rules would have produced, with the numbers behind them.

    python3 tools/proof_replay.py results/rig/2026-10-14/t6h_4 [--weld 0.013 0.210 0.024] [--from T --to T]

Reads /magnet_tip_pose, the ring (/payload/motion_capture_state) and QUAD4's body (/drone_3/motion_capture_state).
The magnet counts as ON from the first sample (the ring span is ON over the whole rejoin). Old bags have no lift-done
signal, so the lift is timed from the contact (--lift-s, his 3 cm at 2 cm/s plus the start).
"""
import argparse
import glob
import math
import sqlite3
import sys

import numpy as np


def quat_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def load(cur, tops, name, quat=False, t_from=None, t_to=None):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    if name not in tops:
        return np.zeros((0, 8 if quat else 4))
    i, ty = tops[name]
    cls = get_message(ty)
    lo = int((t_from or 0) * 1e9)
    hi = int((t_to or 4e9) * 1e9)
    out = []
    for ts, d in cur.execute('select timestamp,data from messages where topic_id=? and timestamp between ? and ? '
                             'order by timestamp', (i, lo, hi)):
        m = deserialize_message(d, cls)
        p, q = m.pose.position, m.pose.orientation
        out.append((ts * 1e-9, p.x, p.y, p.z) + ((q.x, q.y, q.z, q.w) if quat else ()))
    return np.array(out) if out else np.zeros((0, 8 if quat else 4))


def replay(bag, weld, lift_s, t_from=None, t_to=None, **cfg_kw):
    from drone_magnet.comotion_proof import ComotionConfig, ComotionProof
    db = sorted(glob.glob(f'{bag}/*.db3'))
    if not db:
        raise SystemExit(f'no bag in {bag}')
    cur = sqlite3.connect(db[0])
    tops = {n: (i, ty) for i, n, ty in cur.execute('select id,name,type from topics')}
    tip = load(cur, tops, '/magnet_tip_pose', t_from=t_from, t_to=t_to)
    ring = load(cur, tops, '/payload/motion_capture_state', quat=True, t_from=t_from, t_to=t_to)
    body = load(cur, tops, '/drone_3/motion_capture_state', t_from=t_from, t_to=t_to)
    if not len(tip) or not len(ring):
        return []
    proof = ComotionProof(ComotionConfig(lift_fallback_s=lift_s, **cfg_kw))
    proof.magnet(True)
    events = []
    for t, x, y, z in tip:
        j = min(np.searchsorted(ring[:, 0], t), len(ring) - 1)
        if abs(ring[j, 0] - t) > 0.1:
            continue
        rot = quat_rot(ring[j, 4:8])
        plate = ring[j, 1:4] + rot @ np.asarray(weld)
        k = min(np.searchsorted(body[:, 0], t), len(body) - 1) if len(body) else None
        b = body[k, 1:4] if k is not None and abs(body[k, 0] - t) <= 0.1 else None
        ev = proof.step(t, (x, y, z), plate, body=b, rotation=rot) or proof.tick(t)
        if ev:
            rec = proof.report(t)
            if ev == 'failed':
                rec.update({k2: proof.last_fail.get(k2) for k2 in ('reason', 'drift_m', 'xy_m', 'normal_m')})
            events.append((t, ev, rec))
    return events


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('bag', nargs='+')
    ap.add_argument('--weld', nargs=3, type=float, default=(0.013, 0.210, 0.024),
                    help='the weld point in the ring frame (m): the tip marker at contact')
    ap.add_argument('--lift-s', type=float, default=1.8, help='s from contact to his lift done (no signal in old bags)')
    ap.add_argument('--from', dest='t_from', type=float)
    ap.add_argument('--to', dest='t_to', type=float)
    a = ap.parse_args(argv)
    for bag in a.bag:
        events = replay(bag, a.weld, a.lift_s, a.t_from, a.t_to)
        print(f'== {bag}: {sum(e[1] == "contact" for e in events)} contact(s), '
              f'{sum(e[1] == "proven" for e in events)} proven, {sum(e[1] == "failed" for e in events)} failed')
        t0 = None
        for t, ev, rec in events:
            t0 = t0 or t
            extra = (f" reason {rec.get('reason')}" if ev == 'failed' else '')
            print(f'  +{t - t0:6.2f} s  {ev:8s} xy {_mm(rec.get("xy_m"))} normal {_mm(rec.get("normal_m"))} '
                  f'drift {_mm(rec.get("drift_m"))} body-to-tip {rec.get("body_to_tip_m")} '
                  f'l_hang {rec.get("l_hang_m")} comotion {_mm(rec.get("comotion_m"))}{extra}')
    return 0


def _mm(v):
    return '   -  ' if v is None or not math.isfinite(v) else f'{1000 * v:6.1f}mm'


if __name__ == '__main__':
    sys.exit(main())
