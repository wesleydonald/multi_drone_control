#!/usr/bin/env python3
"""His C++ backend's install race, per run: fetch replans.csv from the lab (lab_batch does not sync it) and tally.

    python3 install_tally.py results/2026-10-23/R2695_* [...]

For each run: the C1F.6 C++ log directory named in logs/extra_launch_1.log is copied from
drones:mdc/logs/tejen_dynamic_planner/<dir> into this script's folder (<run id>/), unless already there.
Prints replans, planner-accepted, installed (and when, ROS time), discarded world_version_changed,
final-check failures, and the waits: first replan (the settle) -> first install (prepared) -> second (granted).
"""
import collections
import csv
import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))

for run in sys.argv[1:]:
    rid = os.path.basename(run.rstrip('/')).split('_')[0]
    out = os.path.join(HERE, rid)
    f = os.path.join(out, 'replans.csv')
    if not os.path.exists(f):
        log = open(os.path.join(run, 'logs', 'extra_launch_1.log'), errors='replace').read()
        m = re.search(r'C1F\.6 C\+\+ log directory: \S*/tejen_dynamic_planner/(\S+)', log)
        if not m:
            print(rid, 'no C1F.6 log directory in extra_launch_1.log')
            continue
        os.makedirs(out, exist_ok=True)
        subprocess.run(['scp', '-q', '-o', 'BatchMode=yes',
                        f'drones@drones:mdc/logs/tejen_dynamic_planner/{m.group(1)}/*.csv', out], check=False)
    if not os.path.exists(f):
        print(rid, 'replans.csv not fetched')
        continue
    rows = list(csv.DictReader(open(f)))
    acc = [r for r in rows if r['accepted'] == '1']
    inst = [r for r in rows if r.get('authority_installed') == '1']
    rej = collections.Counter(r.get('installation_rejection', '') for r in acc if r.get('authority_installed') != '1')
    fc = sum('FINAL_CHECK' in r['status'] for r in rows)
    t = [float(rows[0]['request_ros_time'])] + [float(r['request_ros_time']) for r in inst]
    waits = ', '.join(f'{b - a:.1f}' for a, b in zip(t, t[1:]))
    print(f"{rid}: replans {len(rows)}, accepted {len(acc)}, installed {len(inst)} at ROS "
          f"{[r['request_ros_time'] for r in inst]}, discarded {dict(rej)}, final-check failures {fc}; "
          f"waits settle->install->install (s): {waits}; last replan {rows[-1]['request_ros_time']}")
