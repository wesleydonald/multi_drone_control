"""Replans before capture: accepted share, longest gap between accepted plans; replan runtime median / p95."""
import collections, csv, statistics as st, sys
for run in sys.argv[1:]:
    d = list(csv.DictReader(open(f'runs/{run}/driver.csv')))
    auth = next(float(r['t']) for r in d if r['auth'] == '1')
    cap = next((float(r['t']) for r in d if r['captured'] == '1'), float('inf'))
    rows = list(csv.DictReader(open(f'runs/{run}/replans.csv')))
    pre = [r for r in rows if auth <= float(r['request_ros_time']) < cap]
    acc = [float(r['request_ros_time']) for r in pre if r['status'].startswith('ACCEPTED')]
    gaps = [b - a for a, b in zip([auth] + acc, acc)]
    rt = sorted(float(r['replan_runtime_ms']) for r in rows if r['replan_runtime_ms'] not in ('', 'nan'))
    late = sum('LATE' in r['status'] for r in rows)
    print(f'{run}: before capture {len(acc)}/{len(pre)} accepted, longest gap {max(gaps) if gaps else float("nan"):.1f} s; '
          f'replan runtime median {st.median(rt):.0f} ms, p95 {rt[int(0.95 * len(rt))]:.0f} ms; '
          f'over 100 ms {100 * sum(x > 100 for x in rt) / len(rt):.0f}%; LATE {100 * late / len(rows):.0f}% of {len(rows)}')
