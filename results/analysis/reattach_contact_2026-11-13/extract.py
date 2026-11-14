#!/usr/bin/env python3
"""Extract what the contact replay needs from one rig bag into one compressed npz (+ a json of the string topics).

usage: extract.py <bag_dir> <out_prefix>
Arrays (t = bag receive time, s): tip (t, x, y, z), ring (t, x, y, z, qx, qy, qz, qw), q4 (t, x, y, z),
ref (t, x, y, z) (QUAD4's body reference from his planner). JSON: proof records, the string/bool topics' changes.
Adapted from results/analysis/hero_2026-11-13/extract.py."""
import sys, os, json
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
os.makedirs(os.path.dirname(out), exist_ok=True)
POSE = {'/magnet_tip_pose': 'tip', '/payload/motion_capture_state': 'ring', '/drone_3/motion_capture_state': 'q4'}
STR = ('/magnet/attach_proof', '/join_planner/phase', '/magnet/command', '/drone_3/magnet_coil',
       '/tejen/magnet/command', '/drone_3/magnet_ours')
BOOL = ('/magnet/contact', '/join_planner/proof_lift_complete', '/magnet/object_attached', '/tejen/object_attached',
        '/fleet/payload_dropped')
REF = '/join_planner/reference'
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = [t for t in set(POSE) | set(STR) | set(BOOL) | {REF} if t in types]
r.set_filter(rosbag2_py.StorageFilter(topics=want))
cls = {t: get_message(types[t]) for t in want}
arr = {k: [] for k in ('tip', 'ring', 'q4', 'ref')}
proof, changes, last = [], [], {}
while r.has_next():
    t, data, ts = r.read_next()
    m = deserialize_message(data, cls[t])
    s = ts * 1e-9
    if t in POSE:
        p = m.pose.position
        if POSE[t] == 'ring':
            q = m.pose.orientation
            arr['ring'].append((s, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        else:
            arr[POSE[t]].append((s, p.x, p.y, p.z))
    elif t == REF:
        if m.points and m.points[0].transforms:
            p = m.points[0].transforms[0].translation
            arr['ref'].append((s, p.x, p.y, p.z))
    elif t == '/magnet/attach_proof':
        try:
            j = json.loads(m.data)
        except ValueError:
            continue
        j['t'] = s
        proof.append(j)
    else:
        v = m.data if isinstance(m.data, str) else bool(m.data)
        # the string topics: every message (commands are sparse); the bools and the phase: changes only
        if t in ('/magnet/command', '/tejen/magnet/command', '/drone_3/magnet_ours', '/drone_3/magnet_coil'):
            if last.get(t) != v:
                changes.append((s, t, v))
        elif last.get(t) != v:
            changes.append((s, t, v))
        last[t] = v
np.savez_compressed(out + '.npz', **{k: np.array(v, dtype=float).reshape(-1, 8 if k == 'ring' else 4)
                                     for k, v in arr.items()})
with open(out + '.json', 'w') as f:
    json.dump({'bag': bag, 'proof': proof, 'changes': changes}, f)
print(out, {k: len(v) for k, v in arr.items()}, 'proof', len(proof), 'changes', len(changes))
