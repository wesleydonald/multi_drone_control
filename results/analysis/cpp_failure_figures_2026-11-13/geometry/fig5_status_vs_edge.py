"""Fig 5: when and why his replans fail. (a) R4374: each replan's start point (initial_q0) distance to the drop point
over time, coloured by status: the failures begin when QUAD4 reaches the legacy hull's edge. (b) status shares in all
the stall-README runs: the four legacy parameter variants look the same as R4374."""
import collections
import csv
import glob

import numpy as np

from common import *

CAT = [('PREFIX_CORRIDOR_INCOMPATIBLE', C_LEG), ('ACCEPTED (any)', '#27ae60'),
       ('TARGET_TIME_FIXED_POINT_NOT_CONVERGED', '#8e44ad'), ('INCUMBENT_REMAINING_UNSAFE', '#d68910'),
       ('PADDED_CLOSEST_PARTIAL_FINAL_CHECK_FAILED', '#5d6d7e'), ('other', '#bdc3c7')]


def cat(s):
    if 'PREFIX_CORRIDOR_INCOMPATIBLE' in s:
        return 0
    if s.startswith('ACCEPTED'):
        return 1
    if 'TARGET_TIME_FIXED_POINT' in s:
        return 2
    if s.startswith('INCUMBENT_REMAINING_UNSAFE'):
        return 3
    if 'PADDED_CLOSEST_PARTIAL' in s:
        return 4
    return 5


fig, (a, b) = plt.subplots(1, 2, figsize=(15, 5.6), gridspec_kw=dict(width_ratios=[1.1, 1]))
rows = list(csv.DictReader(open(f'{R4374}/replans.csv')))
t = np.array([float(r['request_ros_time']) for r in rows])
q = np.array([[float(r['initial_q0_x']), float(r['initial_q0_y'])] for r in rows])
gx = np.array([float(r['predicted_target_x']) if r['predicted_target_x'] not in ('', 'nan') else np.nan for r in rows])
d = np.hypot(q[:, 0], q[:, 1])           # ring centre at (0, 0) in R4374 (run.csv: |payload xy| < 1 mm)
c = np.array([cat(r['status']) for r in rows])
ev = events(f'{R4374}/events.csv')
t0 = [x for x, e, _ in ev if e == 'PHASE:TRANSIT_TO_DROP_POINT'][0]
for k, (name, col) in enumerate(CAT):
    m = c == k
    if m.any():
        a.scatter(t[m] - t0, d[m], s=9, color=col, label=f'{name} ({100 * m.mean():.0f} %)', zorder=3 if k else 2)
a.axhline(0.675, color='k', ls=':', lw=1)
a.text(150, 0.80, 'legacy hull edge at x = 0: y 0.675-0.70 (Fig 1)', fontsize=8, ha='center')
a.set_xlabel('replan request time since TRANSIT_TO_DROP_POINT (s)')
a.set_ylabel('replan start point (initial_q0) to drop point, horizontal (m)')
a.set_ylim(0, 2.0); a.set_xlim(-2, 215)
a.legend(loc='upper right', fontsize=8, markerscale=2)
a.set_title('R4374 (legacy): once QUAD4 sits on the hull edge, nearly every\nreplan ends PREFIX_CORRIDOR_INCOMPATIBLE')
runs = [('R4374', 'hover, legacy, swing 5'), ('R4378', 'hover, legacy, swing 3'), ('R4379', 'hover, legacy, infl 0.04'),
        ('R4380', 'hover, legacy, body 0.09'), ('R4381', 'hover, legacy, spline 6'),
        ('R4377', 'circle, legacy (reached)'), ('R4382', 'circle, legacy (reached)'),
        ('R4362', 'circle s1, his settings (reached)')]
for j, (rid, lab) in enumerate(runs):
    f = glob.glob(f'{STALL}/{rid}_*/replans.csv')[0]
    st = [cat(r['status']) for r in csv.DictReader(open(f))]
    cnt = collections.Counter(st)
    left = 0
    for k, (name, col) in enumerate(CAT):
        v = 100 * cnt.get(k, 0) / len(st)
        b.barh(len(runs) - 1 - j, v, left=left, color=col, label=name if j == 0 else None)
        if v >= 8:
            b.text(left + v / 2, len(runs) - 1 - j, f'{v:.0f}', ha='center', va='center', fontsize=8, color='white')
        left += v
b.set_yticks(range(len(runs))[::-1])
b.set_yticklabels([f'{r} {l}\n({len(list(csv.DictReader(open(glob.glob(f"{STALL}/{r}_*/replans.csv")[0]))))} replans)'
                   for r, l in runs], fontsize=8)
b.axhline(2.5, color='k', lw=0.8)
b.set_xlim(0, 100); b.set_xlabel('share of replans (%)')
b.legend(loc='upper center', bbox_to_anchor=(0.45, -0.12), ncol=2, fontsize=8)
b.set_title('Status shares: no parameter tried moved the legacy hover failure\n(each value flown once; circles reached '
            'because the drop point moved under QUAD4)')
fig.suptitle('Fig 5. The stall is geometric, not a tuning problem: the failure status appears exactly when QUAD4 reaches '
             'the legacy hull edge, and swing / inflation / body box / spline leave it unchanged.', fontsize=10.5)
fig.tight_layout(rect=(0, 0, 1, 0.94))
save(fig, 'fig5_status_vs_hull_edge.png')
