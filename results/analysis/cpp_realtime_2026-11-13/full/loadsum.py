"""One row per fm9_5 load run: capture, z max, runtime median/p95, LATE, longest gap before capture, fallbacks,
closest distances, stranding. usage (from rigcheck/): python3 full/loadsum.py <run> ..."""
import collections
import csv
import re
import statistics as st
import subprocess
import sys

print('| run | captured (after authority) | body z max | replan runtime median / p95 | LATE | before capture: accepted / replans, longest gap | exec fallbacks | closest body-body / line-tether / line-body (m) | stranding before capture |')
print('|---|---|---|---|---|---|---|---|---|')
for arg in sys.argv[1:]:
    run, flight, off = (arg.split('=')[0], *arg.split('=')[1].split('@')) if '=' in arg else (arg, 'fm9_5', '135')
    d = list(csv.DictReader(open(f'full/runs/{run}/driver.csv')))
    auth = next(float(r['t']) for r in d if r['auth'] == '1')
    cap = next((float(r['t']) for r in d if r['captured'] == '1'), None)
    rows = list(csv.DictReader(open(f'full/runs/{run}/replans.csv')))
    end = cap if cap is not None else float('inf')
    pre = [r for r in rows if auth <= float(r['request_ros_time']) < end]
    acc = [float(r['request_ros_time']) for r in pre if r['status'].startswith('ACCEPTED')]
    last = cap if cap is not None else float(d[-1]['t'])
    gaps = [b - a for a, b in zip([auth] + acc, acc + [last])]
    rt = sorted(float(r['replan_runtime_ms']) for r in rows if r['replan_runtime_ms'] not in ('', 'nan'))
    late = sum('LATE' in r['status'] for r in rows)
    fb = open(f'full/runs/{run}/backend.log').read().count('execution certificate invalid')
    dist = subprocess.run(['python3', 'full/dist.py', run, flight, off], capture_output=True, text=True).stdout
    zmax = re.search(r'z max ([0-9.]+)', dist).group(1)
    bb, lt, lb = (re.search(rf'{k} ([0-9.]+)', dist).group(1) for k in ('body-body', 'line-tether', 'line-body'))
    m = re.search(r'before capture ([0-9.]+) s(?: from \+[0-9.]+ s, min ([0-9.]+) m)?', dist)
    capt = f'yes, {cap - auth:.1f} s' if cap is not None else 'NO'
    if cap is None:
        best = min(d, key=lambda r: float(r['dxy']))
        capt += f' (closest {float(best["dxy"]):.2f} m)'
    print(f'| {run} | {capt} | {zmax} | {st.median(rt):.0f} / {rt[int(0.95 * len(rt))]:.0f} ms | {100 * late / len(rows):.0f}% | '
          f'{len(acc)} / {len(pre)}, {max(gaps):.1f} s | {fb} | {bb} / {lt} / {lb} | {m.group(1)} s{", min " + m.group(2) + " m" if m.group(2) else ""} |')
