"""The zero-clearance windows of the moving-ring replays (Te-Jen's Codex entry 16): per 1 s from 6 s before QUAD4 stopped
to the crossing, the virtual QUAD4 (= his reference), its speed, the closest tether, his backend state (authority, fresh
state, commit end, latest status, execution fallback mode / certified, cooperative ages, world version) and the replans
in that second (statuses, installed). usage: python3 full/zero_window.py <run> <t_cross from driver start, s>"""
import collections
import csv
import sys

run, tc = sys.argv[1], float(sys.argv[2])
d = list(csv.DictReader(open(f'full/runs/{run}/driver.csv')))
b = list(csv.DictReader(open(f'full/runs/{run}/backend.csv')))
rp = list(csv.DictReader(open(f'full/runs/{run}/replans.csv')))
t0 = float(d[0]['t'])
stop = None
for r in d:
    t = float(r['t']) - t0
    if t > tc:
        break
    v = (float(r['vx']) ** 2 + float(r['vy']) ** 2 + float(r['vz']) ** 2) ** 0.5
    stop = None if v > 0.05 else (stop if stop is not None else t)
print(f'## {run}: QUAD4 still from +{stop:.1f} s, crossing at +{tc:.1f} s (driver clock; t0 {t0:.3f})')
print('| t | QUAD4 (= his ref) | speed | dxy to target | closest cable-tether | auth / fresh / has_commit / commit_end-now | latest_status | fallback active / certified / mode / reason | coop ages ms | world v | replans this s: statuses (installed) |')
print('|---|---|---|---|---|---|---|---|---|---|---|')
a = (stop or tc) - 6.0
for k in range(int(a), int(tc) + 1):
    ds = [r for r in d if k <= float(r['t']) - t0 < k + 1]
    bs = [r for r in b if k <= float(r['ros_time']) - t0 < k + 1]
    rs = [r for r in rp if k <= float(r['request_ros_time']) - t0 < k + 1]
    if not ds or not bs:
        continue
    r, x = ds[-1], bs[-1]
    v = (float(r['vx']) ** 2 + float(r['vy']) ** 2 + float(r['vz']) ** 2) ** 0.5
    ce = (float(x['commit_end_s']) - float(x['ros_time'])) if x['commit_end_s'] not in ('', 'nan') else float('nan')
    st = collections.Counter(q['status'] for q in rs)
    inst = sum(q['authority_installed'] == '1' for q in rs)
    print(f"| +{k} | ({float(r['x']):.2f}, {float(r['y']):.2f}, {float(r['z']):.2f}) | {v:.2f} | {r['dxy'][:5]} | "
          f"{min(float(q['min_cable_tether_m']) for q in ds):.2f} | {x['authority_granted']} / {x['state_fresh']} / "
          f"{x['has_commit']} / {ce:.2f} | {x['latest_status'][:40]} | {x['execution_fallback_active']} / "
          f"{x['execution_fallback_certified']} / {x['execution_fallback_mode']} / {x['execution_fallback_reason'][:30]} | "
          f"{x['coop0_age_ms'][:5]}, {x['coop1_age_ms'][:5]} | {x['world_version']} | "
          + ', '.join(f'{s[:34]} x{n}' for s, n in st.most_common(3)) + f' ({inst}) |')
