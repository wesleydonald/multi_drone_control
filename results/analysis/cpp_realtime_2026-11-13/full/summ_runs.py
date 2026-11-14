"""One line per replay run: capture, heights, final offset, his statuses and replan timing."""
import collections
import csv
import math
import os
import statistics as st
import sys

base = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runs')
for run in sys.argv[1:]:
    O = os.path.join(base, run)
    d = list(csv.DictReader(open(f'{O}/driver.csv')))
    t0 = float(d[0]['t'])
    auth = next((float(r['t']) - t0 for r in d if r['auth'] == '1'), float('nan'))
    cap = next((float(r['t']) - t0 for r in d if r['captured'] == '1'), None)
    after = [r for r in d if r['auth'] == '1']
    zmax = max(float(r['z']) for r in d)
    best = min(after, key=lambda r: float(r['dxy']) + abs(float(r['dz']))) if after else None
    last = d[-1]
    rows = list(csv.DictReader(open(f'{O}/replans.csv')))
    c = collections.Counter(r['status'] for r in rows)
    rt = [float(r['replan_runtime_ms']) for r in rows if r['replan_runtime_ms'] not in ('', 'nan')]
    sc = [float(r['safety_check_ms']) for r in rows if r['safety_check_ms'] not in ('', 'nan')]
    ring_sp = [math.hypot(float(r['ring_x']) - float(q['ring_x']), float(r['ring_y']) - float(q['ring_y'])) / 0.1
               for r, q in zip(d[1:], d[:-1])]
    print(f'{run}: authority +{auth:.1f} s; captured {"+%.1f s" % cap if cap is not None else "NO"} '
          f'(sim {float(d[-1]["t"]) - t0:.0f} s); body z max {zmax:.2f}; closest to target dxy {float(best["dxy"]):.2f} '
          f'dz {float(best["dz"]):+.2f} at +{float(best["t"]) - t0:.1f}; end dxy {float(last["dxy"]):.2f} dz {float(last["dz"]):+.2f}; '
          f'replans {len(rows)}: ' + ', '.join(f'{k} {100 * v / len(rows):.0f}%' for k, v in c.most_common(3)) +
          f'; runtime med {st.median(rt):.0f} ms, final-check med {st.median(sc):.1f} ms')
