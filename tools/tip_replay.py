#!/usr/bin/env python3
"""
tools/tip_replay.py -- the magnet-marker release check replayed over a rig flight's bag, offline (Wesley 21 Oct: Part C
flies with the check on only if this is clean over the A/B flights; card 2026-10-21_tip_detect).

    tools/tip_replay.py results/rig/2026-10-22/h3                     # the flight's own geometry
    tools/tip_replay.py results/rig/2026-10-22/f2 --azimuths 0,180 --attach-z 0.02 --tip-m 0.04

It feeds every recorded /drone_<i>/magnet_tip_state sample, in time order, with the ring's and the drones' latest
poses, through the planner's own DissipativeController._tip_check / _tip_learn_step, the ring samples through its
glitch guard _tip_ring_sample (no ROS). Before LAND
(/fleet/command), after the ring first reaches target_z - 0.05. Prints, per drone: armed or not (its learnt offset and
body-to-marker length), the largest deviation after arming against tip_detect_m, and every would-fire (a release
acted on, or a lone marker's logged WOULD-FIRE). Exit status 1 if anything would have fired, 2 if a marker never armed
(the replay then tested nothing for that drone: no pass).

The geometry not typed (cable_len, attach_z, the angles, target_z, the pivot) is the flight's own, from its planner's
params.json beside the bag (<bag>_logs); with neither the replay refuses. A cable length that is not the flight's
leaves every marker unarmed (22 Oct: the 0.55 m default against the 1 m cables).

Typed angles are the sheet's (clockwise, as the launch takes them); the drones are matched to them by position at
the start, as the planner does. Not modelled: detachments, the unload; a flight with a release is not a false-fire
check.
"""
import argparse
import json
import os
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


RUN_BAR_S = 0.05      # critic round 3, N2: no in-band run past both tests may last this long in a flight without a release


def match_slots(events, azimuths_cw, n):
    """slot -> drone as the planner matches them at takeoff (geometry.azimuth_slot_assignment, every permutation for
    n <= 6): the quads may stand in the typed slots in any order (22 Oct: QUAD3 and QUAD4 stood mirrored)."""
    import math
    from mpc_planner.geometry import azimuth_slot_assignment
    ring, pos = None, {}
    for t, kind, i, xyz, extra in events:
        if kind == 'ring':
            ring = (xyz, extra[0])
        elif kind == 'drone' and 0 <= i < n:
            pos[i] = xyz
        if ring is not None and len(pos) == n:
            w, x, y, z = ring[1]
            yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            slot_az = [math.radians(-a) for a in azimuths_cw]
            return list(azimuth_slot_assignment([pos[j] for j in range(n)], ring[0][:2], n, yaw, slot_az=slot_az))
    return list(range(n))


def replay(events, azimuths_cw, attach_z, tip_m=0.04, target_z=1.0, radius=0.225, cable_len=0.55, pivot_z=-0.04,
           rider=None, stats=None):
    """Run the planner's check over the events; returns (node stub, would-fires [(t, drone, line)], max deviation
    after arming per drone). stats, if a dict, gets per drone 'run' (the longest in-band run past both tests: s,
    samples) and 'rejected' (samples off the body-to-marker band after arming)."""
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
            if 'WOULD-FIRE' in m:
                fires.append((f.t, int(re.search(r'/drone_(\d+)', m).group(1)) if re.search(r'/drone_(\d+)', m)
                              else -1, m))
        warn = error = info

    f = SimpleNamespace(slot2drone=list(range(n)), detached=[False] * n, rho=attach_points(n, radius, attach_z, ccw),
                        _tip_detect_m=tip_m, _tip_v0={}, _tip_learn={}, _tip_learn_warned=set(), _tip_run={},
                        _tip_last={}, _tip_raw={}, _tip_lone={}, _tip_unarmed=set(), _tip_hold_until=-1e18,
                        _tip_ring_same=None, _detect_refused=set(), phase='planner', _reconfig_mode='ocp',
                        _land_to_ground=False, _unload=None, takeoff_seen=True, load_state=None, cable_len=cable_len,
                        cable_len_i=[cable_len] * n, pivot_offset=np.array([0.0, 0.0, pivot_z]), lift_z0=None,
                        target_z=target_z, lift_progress=0.0, drone_pos=[None] * n, _drone_t={}, _load_t=None, t=0.0,
                        refs=SimpleNamespace(_t_nom=[0.1 if i == rider else 5.0 for i in range(n)]))
    stats = {} if stats is None else stats
    log = _Log()
    f.get_logger = lambda: log
    def fired(d):                    # the planner would resize d away: it is no longer checked
        fires.append((f.t, d, log.last))
        f.detached[d] = True
    f._on_unannounced = fired
    for name in ('_tip_learn_step', '_tip_ring_sample', '_tip_rod'):
        setattr(f, name, (lambda fn: lambda *a: fn(f, *a))(getattr(D, name)))
    f.slot2drone = match_slots(events, azimuths_cw, n)
    lifted = False
    for t, kind, i, xyz, extra in events:
        f.t = t
        if kind == 'land':
            f._land_to_ground = True
        elif kind == 'ring':
            q, v = extra
            prev = None if f.load_state is None else f.load_state[0:7].copy()
            f.load_state = np.r_[xyz, q, v, np.zeros(3)]
            f._load_t = t
            if prev is not None:
                D._tip_ring_sample(f, prev, f.load_state[0:7], t)
            if f.lift_z0 is None:
                f.lift_z0 = float(xyz[2])
            lifted = lifted or xyz[2] >= target_z - 0.05
            f.lift_progress = (target_z - f.lift_z0) if lifted else 0.0
        elif kind == 'drone' and 0 <= i < n:
            f.drone_pos[i] = xyz
            f._drone_t[i] = t
        elif kind == 'tip' and 0 <= i < n and f.load_state is not None:
            st = stats.setdefault(i, {'run': (0.0, 0), 'rejected': 0})
            if i in f._tip_v0 and f.drone_pos[i] is not None and not f.detached[i]:
                lo, hi = f._tip_v0[i][3]
                st['rejected'] += not lo <= float(np.linalg.norm(xyz - f.drone_pos[i])) <= hi
            D._tip_check(f, i, xyz, t)
            if i in f._tip_run:
                t0, ts, k = f._tip_run[i]
                st['run'] = max(st['run'], (ts - t0, k))
            if i in f._tip_v0 and i in f._tip_last and f._tip_last[i][0] == t:
                devmax[i] = max(devmax.get(i, 0.0), f._tip_last[i][1])
    return f, fires, devmax


def flight_params(bag):
    """(params, session dir) of the flight's planner (its last session under <bag>_logs), or (None, None)."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import run_logs
    logs = os.path.normpath(bag) + '_logs'
    dirs = [d for kind in ('mpc_planner', 'dissipative_planner') for d in run_logs.node_dirs(logs, kind)
            if os.path.exists(os.path.join(d, 'params.json'))]
    if not dirs:
        return None, None
    d = max(dirs, key=lambda x: run_logs.stamp_of(x) or '')
    with open(os.path.join(d, 'params.json')) as fh:
        return json.load(fh), d


def geometry(a, prm):
    """{cable_len, attach_z, azimuths (clockwise), target_z, pivot_z}: typed, else the flight's params.json."""
    def flown(key, flag):
        if prm is None or prm.get(key) in (None, ''):
            sys.exit(f'{flag} not typed and no planner params.json beside the bag ({a.bag}_logs): type {flag}')
        return prm[key]
    if a.azimuths is not None:
        az = [float(v) for v in a.azimuths.split(',')]
    else:
        n = int(flown('num_drones', '--azimuths'))
        ccw = str(prm.get('attach_azimuths_deg') or '').strip()       # as the node reads it: counter-clockwise
        az = [(-float(v)) % 360.0 for v in ccw.split(',')] if ccw else [(-360.0 * k / n) % 360.0 for k in range(n)]
    pivot = prm.get('pivot_offset') if prm else None
    if isinstance(pivot, str):                     # params.json holds some lists as their string
        pivot = json.loads(pivot)
    return {'cable_len': float(a.cable_len if a.cable_len is not None else flown('cable_len', '--cable-len')),
            'attach_z': float(a.attach_z if a.attach_z is not None else flown('attach_z', '--attach-z')),
            'azimuths': az,
            'target_z': float(a.target_z if a.target_z is not None else (prm or {}).get('target_z', 1.0)),
            'pivot_z': float(pivot[2]) if pivot else -0.04}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='the flight\'s bag directory (results/rig/<date>/<flight>)')
    ap.add_argument('--azimuths', help='the typed attach_azimuths_deg (clockwise), e.g. 0,240,120 (default: the '
                    'flight\'s)')
    ap.add_argument('--attach-z', type=float, help='the typed attach_z (default: the flight\'s)')
    ap.add_argument('--tip-m', type=float, default=0.04)
    ap.add_argument('--target-z', type=float, help='default: the flight\'s, else 1.0')
    ap.add_argument('--cable-len', type=float, help='the typed cable_len (default: the flight\'s)')
    ap.add_argument('--rider', type=int, help='the drone index carrying ~0 (the T\'s quad opposite the gap): its band '
                    'reaches up to the rod, as the planner\'s')
    a = ap.parse_args(argv)
    prm, src = flight_params(a.bag)
    geo = geometry(a, prm)
    ev = read(a.bag)
    n_tip = sum(1 for e in ev if e[1] == 'tip')
    print(f'{a.bag}: {n_tip} magnet-marker samples, {sum(1 for e in ev if e[1] == "ring")} ring poses; cable_len '
          f'{geo["cable_len"]:g}, attach_z {geo["attach_z"]:g}, angles {",".join(f"{v:g}" for v in geo["azimuths"])} '
          f'(clockwise), target_z {geo["target_z"]:g}' + (f' (untyped ones from {src})' if src else ''))
    if not n_tip:
        sys.exit('no /drone_<i>/magnet_tip_state in the bag: were the markers routed (mocap_extra_bodies)?')
    az = geo['azimuths']
    stats = {}
    f, fires, devmax = replay(ev, az, geo['attach_z'], a.tip_m, geo['target_z'], cable_len=geo['cable_len'],
                              pivot_z=geo['pivot_z'], rider=a.rider, stats=stats)
    long_runs = 0
    for d in range(len(az)):
        state = ('armed' if d in f._tip_v0 else 'NOT armed' if d in f._tip_unarmed else 'never learnt (its marker '
                 'never held still for 1 s after the lift?)')
        st = stats.get(d, {'run': (0.0, 0), 'rejected': 0})
        long_runs += st['run'][0] >= RUN_BAR_S
        print(f'  drone {d}: {state}; largest deviation after arming {100.0 * devmax.get(d, float("nan")):.1f} cm '
              f'(fires at {100.0 * a.tip_m:.0f} cm); longest in-band run past both tests {1000.0 * st["run"][0]:.0f} ms '
              f'({st["run"][1]} samples; bar < {1000.0 * RUN_BAR_S:.0f} ms: '
              f'{"PASS" if st["run"][0] < RUN_BAR_S else "FAIL"}); {st["rejected"]} samples off the band')
    print(f'WOULD-FIRES: {len(fires)}' + ''.join(f'\n  t={t:.2f} drone {d}: {m}' for t, d, m in fires))
    print(f'RUNS OVER {1000.0 * RUN_BAR_S:.0f} ms: {long_runs}')
    unarmed = [d for d in range(len(az)) if d not in f._tip_v0]
    if fires or long_runs:
        sys.exit(1)
    if unarmed:
        print(f'NOT A PASS: drone(s) {unarmed} never armed, so nothing was tested for them (a cable_len that is not the '
              f'flight\'s, a marker not routed, or a magnet off its plate)')
        sys.exit(2)
    sys.exit(0)


if __name__ == '__main__':
    main()
