#!/usr/bin/env python3
"""Extract the hero run's tip, object, ring, QUAD4, reference and magnet topics from a rig bag to CSVs.
usage: extract.py <bag_dir> <out_dir>"""
import sys, os, csv
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
POSE = {'/magnet_tip_pose': 'tip', '/pickup/motion_capture_state': 'obj', '/payload/motion_capture_state': 'ring',
        '/drone_3/motion_capture_state': 'q4'}
BOOL = {'/magnet/contact': 'contact', '/tejen/object_attached': 'tj_obj_att', '/magnet/object_attached': 'obj_att',
        '/drone_3/magnet_coil': 'coil'}
STR = {'/magnet/attach_proof': 'proof'}
REF = '/join_planner/reference'
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = set(POSE) | set(BOOL) | set(STR) | {REF}
r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in want if t in types]))
cls = {t: get_message(types[t]) for t in want if t in types}
rows = {k: [] for k in list(POSE.values()) + ['bool', 'proof', 'ref']}
while r.has_next():
    t, data, ts = r.read_next()
    m = deserialize_message(data, cls[t]); s = ts * 1e-9
    if t in POSE:
        p = m.pose.position; q = m.pose.orientation
        rows[POSE[t]].append((s, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    elif t in BOOL:
        rows['bool'].append((s, BOOL[t], int(m.data) if isinstance(m.data, bool) else str(m.data)))
    elif t in STR:
        rows['proof'].append((s, m.data.replace('\n', ' ')))
    elif t == REF and m.points and m.points[0].transforms:
        p = m.points[0].transforms[0].translation
        rows['ref'].append((s, p.x, p.y, p.z))
hdr = {'bool': ('t', 'topic', 'v'), 'proof': ('t', 'text'), 'ref': ('t', 'x', 'y', 'z')}
for k, v in rows.items():
    with open(os.path.join(out, k + '.csv'), 'w', newline='') as f:
        w = csv.writer(f, lineterminator='\n'); w.writerow(hdr.get(k, ('t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))); w.writerows(v)
    print(k, len(v))
