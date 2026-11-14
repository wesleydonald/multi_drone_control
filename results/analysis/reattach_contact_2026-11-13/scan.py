#!/usr/bin/env python3
"""Scan every rig bag with /magnet/attach_proof for contact events (a proof state 'prove' or 'attached', or
/magnet/contact True). usage: scan.py <out_csv> <bag_dir>..."""
import sys, json, csv, os
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

out = sys.argv[1]
rows = []
for bag in sys.argv[2:]:
    try:
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    except Exception as e:  # noqa: BLE001
        rows.append((bag, 'open_failed', str(e)[:80], '', '', '', ''))
        continue
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    want = [t for t in ('/magnet/attach_proof', '/magnet/contact') if t in types]
    r.set_filter(rosbag2_py.StorageFilter(topics=want))
    cls = {t: get_message(types[t]) for t in want}
    states, max_tries, contact_true, n_on = set(), 0, 0, 0
    while r.has_next():
        t, data, ts = r.read_next()
        m = deserialize_message(data, cls[t])
        if t == '/magnet/contact':
            contact_true += int(bool(m.data))
        else:
            try:
                j = json.loads(m.data)
            except ValueError:
                continue
            states.add(j.get('state'))
            max_tries = max(max_tries, int(j.get('tries') or 0))
            n_on += int(j.get('state') != 'off')
    rows.append((bag, 'ok', '|'.join(sorted(s for s in states if s)), max_tries, contact_true, n_on, types.get(
        '/magnet_tip_pose', '')))
    print(rows[-1], flush=True)
with open(out, 'w', newline='') as f:
    w = csv.writer(f, lineterminator='\n')
    w.writerow(('bag', 'status', 'states', 'max_tries', 'contact_true_msgs', 'proof_msgs_not_off', 'tip_type'))
    w.writerows(rows)
