"""Extract a flight's poses (20 Hz samples) and phase timeline from a 12 Nov rig bag.
usage: python3 extract.py <bag_dir> <out_prefix>
writes <out>_poses.csv (t, phase, d0..d3 pos+vel, ring pos+quat+vel, tip, obj) and <out>_events.txt"""
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

bag, out = sys.argv[1], sys.argv[2]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
mocap = ['/drone_0/motion_capture_state', '/drone_1/motion_capture_state', '/drone_2/motion_capture_state',
         '/drone_3/motion_capture_state', '/payload/motion_capture_state', '/pickup/motion_capture_state']
other = ['/join_planner/phase', '/payload/trajectory_state', '/tejen/magnet_tip_pose', '/fleet/command',
         '/fleet/drop', '/magnet/command', '/tejen/magnet/command', '/join_planner/reference']
want = [t for t in mocap + other if t in types]
r.set_filter(rosbag2_py.StorageFilter(topics=want))
cls = {t: get_message(types[t]) for t in want}
last = {}
phase = ''
t0 = None
next_sample = None
ev = open(out + '_events.txt', 'w')
f = open(out + '_poses.csv', 'w')
cols = ['t', 'phase']
for i in range(4):
    cols += [f'd{i}_{k}' for k in ('x', 'y', 'z', 'vx', 'vy', 'vz')]
cols += ['r_x', 'r_y', 'r_z', 'r_qw', 'r_qx', 'r_qy', 'r_qz', 'r_vx', 'r_vy', 'r_vz', 'tip_x', 'tip_y', 'tip_z',
         'o_x', 'o_y', 'o_z', 'ref_x', 'ref_y', 'ref_z']
f.write(','.join(cols) + '\n')
traj_last = None
stop_at = None
ref_fields_printed = False
while r.has_next():
    topic, data, t = r.read_next()
    m = deserialize_message(data, cls[topic])
    if t0 is None:
        t0 = t
        next_sample = t
    ts = (t - t0) / 1e9
    if topic == '/join_planner/phase':
        if m.data != phase:
            phase = m.data
            ev.write(f'{ts:8.2f} PHASE {phase}\n')
    elif topic == '/payload/trajectory_state':
        s = m.data
        try:
            d = json.loads(s)
            key = (d.get('kind'), d.get('running'), d.get('speed'), d.get('radius'))
        except ValueError:
            key = s
        if key != traj_last:
            traj_last = key
            ev.write(f'{ts:8.2f} TRAJ {s}\n')
    elif topic in ('/fleet/command', '/fleet/drop', '/magnet/command', '/tejen/magnet/command'):
        ev.write(f'{ts:8.2f} {topic} {str(m)[:160]}\n')
    elif topic == '/tejen/magnet_tip_pose':
        p = m.pose.position if hasattr(m, 'pose') else m.position
        last['tip'] = (p.x, p.y, p.z)
    elif topic == '/join_planner/reference':
        if not ref_fields_printed:
            ev.write(f'REFTYPE {types[topic]} {list(m.get_fields_and_field_types().items())[:6]}\n')
            ref_fields_printed = True
        try:
            p = m.pose.position
            last['ref'] = (p.x, p.y, p.z)
        except AttributeError:
            try:
                p = m.position
                last['ref'] = (p.x, p.y, p.z)
            except AttributeError:
                pass
    else:
        p, v = m.pose.position, m.twist.linear
        q = m.pose.orientation
        last[topic] = (p.x, p.y, p.z, v.x, v.y, v.z, q.w, q.x, q.y, q.z)
    if phase in ('TRANSIT_TO_REATTACH', 'RING_ATTACHED') and stop_at is None:
        stop_at = ts + 3.0
    if stop_at is not None and ts > stop_at:
        break
    if t >= next_sample and all(k in last for k in mocap[:5]):
        next_sample = t + 50_000_000
        row = [f'{ts:.3f}', phase]
        for i in range(4):
            row += [f'{x:.4f}' for x in last[mocap[i]][:6]]
        rp = last[mocap[4]]
        row += [f'{x:.4f}' for x in rp[:3]] + [f'{x:.5f}' for x in rp[6:10]] + [f'{x:.4f}' for x in rp[3:6]]
        for k, n in (('tip', 3), (mocap[5], 3), ('ref', 3)):
            row += [f'{x:.4f}' for x in last[k][:n]] if k in last else [''] * n
        f.write(','.join(row) + '\n')
f.close()
ev.close()
