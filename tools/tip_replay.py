#!/usr/bin/env python3
"""
tools/tip_replay.py -- the magnet-marker release check replayed over a rig flight's bag, offline (Wesley 21 Oct: Part C
flies with the check on only if this is clean over the A/B flights; card 2026-10-21_tip_detect).

    tools/tip_replay.py results/rig/2026-10-22/h3 --azimuths 0,240,120 --attach-z 0.02
    tools/tip_replay.py results/rig/2026-10-22/f2 --azimuths 0,180 --attach-z 0.02 --tip-m 0.04

It feeds every recorded /drone_<i>/magnet_tip_state sample, in time order, with the ring's and the drones' latest
poses, through the planner's own DissipativeController._tip_check / _tip_learn_step (no ROS). Before LAND
(/fleet/command), after the ring first reaches target_z - 0.05. Prints, per drone: armed or not (its learnt offset and
body-to-marker length), the largest deviation after arming against tip_detect_m, and every would-fire. Exit status 1
if anything would have fired.

Typed angles are the sheet's (clockwise, as the launch takes them); slot i is drone i in that order. Not modelled:
the planner's slot re-assignment, detachments, the unload; a flight with a release is not a false-fire check.
"""
import argparse
import re
import sys
from types import SimpleNamespace

import numpy as np


def read(bag):
    """[(t, kind, index, xyz, extra)] in time order: kind 'ring' (extra: quat wxyz, vel), 'drone', 'tip', 'land'."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = []
    while r.has_next():
        topic, data, ts = r.read_next()
        t = ts * 1e-9
        if topic == '/payload/motion_capture_state':
            m = deserialize_message(data, get_message(types[topic]))
            p, o, v = m.pose.position, m.pose.orientation, m.twist.linear
            out.append((t, 'ring', -1, np.array([p.x, p.y, p.z]), (np.array([o.w, o.x, o.y, o.z]),
                                                                   np.array([v.x, v.y, v.z]))))
            continue
        mt = re.fullmatch(r'/drone_(\d+)/(motion_capture_state|magnet_tip_state)', topic)
        if mt:
            m = deserialize_message(data, get_message(types[topic]))
            p = m.pose.position
            out.append((t, 'drone' if mt.group(2) == 'motion_capture_state' else 'tip', int(mt.group(1)),
                        np.array([p.x, p.y, p.z]), None))
        elif topic == '/fleet/command':
            m = deserialize_message(data, get_message(types[topic]))
            if str(getattr(m, 'data', '')).upper().startswith('LAND'):
                out.append((t, 'land', -1, None, None))
    out.sort(key=lambda e: e[0])
    return out


def replay(events, azimuths_cw, attach_z, tip_m=0.04, target_z=1.0, radius=0.225, cable_len=0.55, pivot_z=-0.04):
    """Run the planner's check over the events; returns (node stub, would-fires [(t, drone, line)], max deviation
    after arming per drone)."""
    from dissipative_planner.dissipative_node import DissipativeController as D
    from mpc_planner.geometry import attach_points
    n = len(azimuths_cw)
    ccw = ','.join(f'{(-a) % 360.0:g}' for a in azimuths_cw)
    fires, devmax = [], {}

    class _Log:
        def __init__(self):
            self.last = ''

        def info(self, m, *a, **k):
            self.last = m
            print('   ', m)
        warn = error = info

    f = SimpleNamespace(slot2drone=list(range(n)), detached=[False] * n, rho=attach_points(n, radius, attach_z, ccw),
                        _tip_detect_m=tip_m, _tip_v0={}, _tip_learn={}, _tip_run={}, _tip_last={}, _tip_unarmed=set(),
                        _detect_refused=set(), phase='planner', _reconfig_mode='ocp', _land_to_ground=False,
                        _unload=None, takeoff_seen=True, load_state=None, cable_len=cable_len,
                        cable_len_i=[cable_len] * n, pivot_offset=np.array([0.0, 0.0, pivot_z]), lift_z0=None,
                        target_z=target_z, lift_progress=0.0, drone_pos=[None] * n, _drone_t={}, _load_t=None, t=0.0)
    log = _Log()
    f.get_logger = lambda: log
    def fired(d):                    # the planner would resize d away: it is no longer checked
        fires.append((f.t, d, log.last))
        f.detached[d] = True
    f._on_unannounced = fired
    f._tip_learn_step = lambda d, sl, v, b, now: D._tip_learn_step(f, d, sl, v, b, now)
    lifted = False
    for t, kind, i, xyz, extra in events:
        f.t = t
        if kind == 'land':
            f._land_to_ground = True
        elif kind == 'ring':
            q, v = extra
            f.load_state = np.r_[xyz, q, v, np.zeros(3)]
            f._load_t = t
            if f.lift_z0 is None:
                f.lift_z0 = float(xyz[2])
            lifted = lifted or xyz[2] >= target_z - 0.05
            f.lift_progress = (target_z - f.lift_z0) if lifted else 0.0
        elif kind == 'drone' and 0 <= i < n:
            f.drone_pos[i] = xyz
            f._drone_t[i] = t
        elif kind == 'tip' and 0 <= i < n and f.load_state is not None:
            D._tip_check(f, i, xyz, t)
            if i in f._tip_v0 and i in f._tip_last and f._tip_last[i][0] == t:
                devmax[i] = max(devmax.get(i, 0.0), f._tip_last[i][1])
    return f, fires, devmax


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='the flight\'s bag directory (results/rig/<date>/<flight>)')
    ap.add_argument('--azimuths', required=True, help='the typed attach_azimuths_deg (clockwise), e.g. 0,240,120')
    ap.add_argument('--attach-z', type=float, required=True, help='the typed attach_z (0.02 after the CoM fix)')
    ap.add_argument('--tip-m', type=float, default=0.04)
    ap.add_argument('--target-z', type=float, default=1.0)
    ap.add_argument('--cable-len', type=float, default=0.55)
    a = ap.parse_args()
    ev = read(a.bag)
    n_tip = sum(1 for e in ev if e[1] == 'tip')
    print(f'{a.bag}: {n_tip} magnet-marker samples, {sum(1 for e in ev if e[1] == "ring")} ring poses')
    if not n_tip:
        sys.exit('no /drone_<i>/magnet_tip_state in the bag: were the markers routed (mocap_extra_bodies)?')
    az = [float(v) for v in a.azimuths.split(',')]
    f, fires, devmax = replay(ev, az, a.attach_z, a.tip_m, a.target_z, cable_len=a.cable_len)
    for d in range(len(az)):
        state = ('armed' if d in f._tip_v0 else 'NOT armed' if d in f._tip_unarmed else 'never learnt (no still '
                 'second after the lift?)')
        print(f'  drone {d}: {state}; largest deviation after arming {100.0 * devmax.get(d, float("nan")):.1f} cm '
              f'(fires at {100.0 * a.tip_m:.0f} cm)')
    print(f'WOULD-FIRES: {len(fires)}' + ''.join(f'\n  t={t:.2f} drone {d}: {m}' for t, d, m in fires))
    sys.exit(1 if fires else 0)


if __name__ == '__main__':
    main()
