#!/usr/bin/env python3
"""Pull the pickup window of every rig flight with a pickup out of its bag (tip, object, QUAD4, reference, coil, attach).

The window: 15 s before the first DESCEND_TO_PICKUP to 25 s after the last pickup phase ends (from the flight's log).
Combined-launch bags carry /pickup/motion_capture_state and /drone_3/motion_capture_state; the one-quad bags (q4p,
q4d_*) carry /irl/pickup_object/pose (PoseArray, [0]) and /motion_capture_state, and his /magnet/command as the coil.
usage: extract.py [day/flight ...]   (default: every flight in FLIGHTS)
writes extract/<day>_<flight>/{tip,obj,q4,ref,coil,att}.csv.gz"""
import sys, os, re, csv, gzip, glob
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = '/home/wesley/multi_drone_control/results/rig'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'extract')
FLIGHTS = ['2026-10-14/t5_4', '2026-10-14/t5_5', '2026-10-14/t6', '2026-10-29/k1_8',
           '2026-11-04/k1m', '2026-11-04/k1m_2', '2026-11-04/k1m_3', '2026-11-04/k1m_4',
           '2026-11-05/k1d', '2026-11-05/fm9y_2', '2026-11-05/k1o_2', '2026-11-05/k1ob_2', '2026-11-06/q4p',
           '2026-11-11/fm9c', '2026-11-11/fm9c_3', '2026-11-11/fm9c_4', '2026-11-11/fm9c_6', '2026-11-11/k1zo',
           '2026-11-11/k1zo_3', '2026-11-12/fm9_2', '2026-11-12/fm9_5', '2026-11-12/fm9c_3', '2026-11-12/fm9c_5',
           '2026-11-12/q4d_3', '2026-11-12/q4d_4', '2026-11-12/q4d_7', '2026-11-13/fm9c_2']
POSE = {'/magnet_tip_pose': 'tip', '/pickup/motion_capture_state': 'obj', '/irl/pickup_object/pose': 'obj',
        '/drone_3/motion_capture_state': 'q4', '/motion_capture_state': 'q4'}
STR = {'/drone_3/magnet_coil': 'coil', '/tejen/magnet/command': 'his_cmd', '/magnet/command': 'his_cmd'}
BOOL = {'/tejen/object_attached': 'his_att', '/magnet/object_attached': 'mgr_att'}
REF = '/join_planner/reference'
PICK = ('DESCEND_TO_PICKUP', 'MAGNET_ATTACH_WAIT', 'LIFT_OBJECT')


def window(log):
    ts = []
    for line in open(log, errors='replace'):
        m = re.search(r'\[(\d+\.\d+)\] \[online_join_planner\]: Planner phase: (\S+) -> (\S+)\.', line)
        if m and (m[2] in PICK or m[3] in PICK):
            ts.append(float(m[1]))
    return min(ts) - 15.0, max(ts) + 25.0


def extract(spec):
    day, fl = spec.split('/')
    a, b = window(f'{ROOT}/{day}/{fl}.log')
    bag = glob.glob(f'{ROOT}/{day}/{fl}/*.db3')[0]
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=os.path.dirname(bag), storage_id='sqlite3'),
           rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    # the one-quad bags have /motion_capture_state for QUAD4; the combined ones use /drone_3/...
    want = [t for t in set(POSE) | set(STR) | set(BOOL) | {REF} if t in types]
    if '/drone_3/motion_capture_state' in types and '/motion_capture_state' in want:
        want.remove('/motion_capture_state')
    if '/drone_3/magnet_coil' in types and '/magnet/command' in want:
        want.remove('/magnet/command')
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    rows = {k: [] for k in ('tip', 'obj', 'q4', 'ref', 'coil', 'his_cmd', 'his_att', 'mgr_att')}
    while r.has_next():
        t, data, ts = r.read_next()
        s = ts * 1e-9
        if s < a or s > b:
            continue
        m = deserialize_message(data, cls[t])
        if t in POSE:
            pose = m.poses[0] if hasattr(m, 'poses') else m.pose
            if hasattr(m, 'poses') and not m.poses:
                continue
            p, q = pose.position, pose.orientation
            rows[POSE[t]].append((f'{s:.6f}', p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        elif t in STR:
            rows[STR[t]].append((f'{s:.6f}', m.data.strip()))
        elif t in BOOL:
            rows[BOOL[t]].append((f'{s:.6f}', int(m.data)))
        elif t == REF and m.points and m.points[0].transforms:
            p = m.points[0].transforms[0].translation
            rows['ref'].append((f'{s:.6f}', p.x, p.y, p.z))
    d = os.path.join(OUT, f'{day}_{fl}')
    os.makedirs(d, exist_ok=True)
    hdr = {'ref': ('t', 'x', 'y', 'z'), 'coil': ('t', 'v'), 'his_cmd': ('t', 'v'), 'his_att': ('t', 'v'),
           'mgr_att': ('t', 'v')}
    for k, v in rows.items():
        with gzip.open(os.path.join(d, k + '.csv.gz'), 'wt', newline='') as f:
            w = csv.writer(f, lineterminator='\n')
            w.writerow(hdr.get(k, ('t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')))
            w.writerows(v)
    print(spec, {k: len(v) for k, v in rows.items()}, flush=True)


for spec in (sys.argv[1:] or FLIGHTS):
    extract(spec)
