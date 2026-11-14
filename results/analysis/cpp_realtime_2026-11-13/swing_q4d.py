"""QUAD4 cable swing (body -> magnet tip from vertical) in the one-quad q4d flights, where his C++ flew the loaded
transit (TRANSIT_TO_DROP_POINT). usage: python3 swing_q4d.py <bag dir> ..."""
import math
import statistics as st
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

for bag in sys.argv[1:]:
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in ('/motion_capture_state', '/magnet_tip_pose', '/join_planner/phase') if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    phase, body, tip = '', None, None
    out = {}
    lift = []
    while r.has_next():
        topic, data, t = r.read_next()
        m = deserialize_message(data, cls[topic])
        if topic == '/join_planner/phase':
            phase = m.data
        elif topic == '/motion_capture_state':
            p = m.pose.position
            body = (p.x, p.y, p.z, m.twist.linear.x, m.twist.linear.y, m.twist.linear.z)
        else:
            p = m.pose.position if hasattr(m, 'pose') else m.position
            tip = (p.x, p.y, p.z)
            if body is None:
                continue
            dx, dy, dz = tip[0] - body[0], tip[1] - body[1], body[2] - tip[2]
            if dz < 0.9:
                continue
            a = math.degrees(math.atan2(math.hypot(dx, dy), dz))
            out.setdefault(phase, []).append((a, math.hypot(body[3], body[4])))
            if math.hypot(body[3], body[4]) < 0.03 and abs(body[5]) < 0.03:
                lift.append((dx, dy))
    o = (st.median(x for x, _ in lift), st.median(y for _, y in lift)) if lift else (0, 0)
    print(f'{bag.split("/")[-1]}: still loaded samples {len(lift)}, static offset ({o[0] * 100:+.1f}, {o[1] * 100:+.1f}) cm')
    for ph in ('TRANSIT_TO_DROP_POINT', 'SETTLE_ABOVE_DROP_POINT'):
        v = sorted(a for a, _ in out.get(ph, []))
        if v:
            sp = max(s for _, s in out[ph])
            print(f'   {ph}: n {len(v)}, swing median {st.median(v):.1f}, p95 {v[int(.95 * len(v))]:.1f}, max {v[-1]:.1f} deg, '
                  f'> 3 deg {100 * sum(x > 3 for x in v) / len(v):.0f}%, > 5 deg {100 * sum(x > 5 for x in v) / len(v):.0f}%; QUAD4 speed max {sp:.2f} m/s')
