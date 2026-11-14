"""Table rows for the coordinator: per run, capture, z max, closest approaches, status shares, world version rate.
usage (from rigcheck/): python3 full/table.py <tag> [<tag> ...]   ('' for the baseline)"""
import collections
import csv
import re
import subprocess
import sys

FLIGHTS = [('fm9_5', 135), ('fm9hc_2', 55), ('fm9hc_4', 70), ('fm9hc_5', 47), ('nro', 66)]
print('| run | captured (from authority) | transit body z max | closest QUAD4 to a carrier (body-body / hanging line-tether / line-body) | WORLD_CHANGED | ACCEPTED* | LATE | other top status | exec fallbacks | world versions/s | stranding before capture (s, min m) | stranding whole (s, min m) | binary |')
print('|---|---|---|---|---|---|---|---|---|---|---|---|---|')
jobs = []
for tag in sys.argv[1:]:
    if '=' in tag:      # run=flight@bag_offset
        run, rest = tag.split('=')
        f, off = rest.split('@')
        jobs.append((run, f, float(off)))
        continue
    tag = '' if tag == 'base' else tag
    jobs += [(f + tag, f, off) for f, off in FLIGHTS]
for run, f, off in jobs:
    if True:
        try:
            d = list(csv.DictReader(open(f'full/runs/{run}/driver.csv')))
            rows = list(csv.DictReader(open(f'full/runs/{run}/replans.csv')))
        except FileNotFoundError:
            continue
        t0 = float(d[0]['t'])
        auth = next((float(r['t']) - t0 for r in d if r['auth'] == '1'), None)
        cap = next((float(r['t']) - t0 for r in d if r['captured'] == '1'), None)
        dist = subprocess.run(['python3', 'full/dist.py', run, f, str(off)], capture_output=True, text=True).stdout
        zmax = re.search(r'z max ([0-9.]+)', dist).group(1)
        bb = re.search(r'body-body ([0-9.]+)', dist).group(1)
        lt = re.search(r'line-tether ([0-9.]+)', dist).group(1)
        lb = re.search(r'line-body ([0-9.]+)', dist).group(1)
        n = len(rows)
        c = collections.Counter(r['status'] for r in rows)
        wc = sum(v for k, v in c.items() if 'WORLD_CHANGED' in k)
        acc = sum(v for k, v in c.items() if k.startswith('ACCEPTED'))
        late = sum(v for k, v in c.items() if 'LATE' in k)
        other = [(k, v) for k, v in c.most_common() if 'WORLD_CHANGED' not in k and not k.startswith('ACCEPTED') and 'LATE' not in k]
        oth = f'{other[0][0][:48]} {100 * other[0][1] / n:.0f}%' if other else '-'
        fb = open(f'full/runs/{run}/backend.log').read().count('execution certificate invalid')
        v = [(float(r['request_ros_time']), float(r['planning_world_version'])) for r in rows
             if r['planning_world_version'] not in ('', 'nan')]
        v = [x for x in v if x[1] > 0]
        rate = (v[-1][1] - v[0][1]) / (v[-1][0] - v[0][0]) if len(v) > 2 else float('nan')
        capt = f'yes, +{cap - auth:.1f} s' if cap is not None else 'no'
        m1 = re.search(r'longest ([0-9.]+) s(?: from \+[0-9.]+ s, min ([0-9.]+) m)?; before capture ([0-9.]+) s(?: from \+[0-9.]+ s, min ([0-9.]+) m)?', dist)
        sw = f'{m1.group(1)}, {m1.group(2) or "-"}' if m1 else '?'
        sb = f'{m1.group(3)}, {m1.group(4) or "-"}' if m1 else '?'
        # which backend binary: v0 = before 01:52:46, v1 = 01:52:46 (period param), v2 = 02:03:52 (Te-Jen's review fixes)
        stamp = float(re.search(r'\[(1[0-9]{9}\.[0-9]+)\]', open(f'full/runs/{run}/backend.log').read()).group(1))
        binl = 'v0' if stamp < 1794495166 else ('v1' if stamp < 1794495832 else 'v2')
        try:
            binl += ' (' + open(f'full/runs/{run}/binary.txt').readline().split()[1][:8] + ')'
        except (FileNotFoundError, IndexError):
            pass
        print(f'| {run} | {capt} | {zmax} | {bb} / {lt} / {lb} | {100 * wc / n:.0f}% | {100 * acc / n:.0f}% | '
              f'{100 * late / n:.0f}% | {oth} | {fb} | {rate:.0f} | {sb} | {sw} | {binl} |')
