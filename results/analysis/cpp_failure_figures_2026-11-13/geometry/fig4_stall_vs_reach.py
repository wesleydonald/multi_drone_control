"""Fig 4: R4374 (legacy_square, stalled) vs R4383 / R4392 (outer_octagon, reached the drop point), twin hover from
9 o'clock, same settings otherwise. QUAD4's path (top view), its distance to the drop point, and R4374's replan
statuses. R4378-R4381 (legacy, other parameter values) have no run.csv: their last replan start (initial_q0) marks
where they stood."""
import collections
import csv
import glob

import numpy as np

from common import *

CASES = [('R4374 legacy_square', R4374, C_LEG), ('R4383 outer_octagon', R4383, C_OCT),
         ('R4392 outer_octagon (30 steps)', R4392, '#5dade2')]


def transit(path):
    ev = events(f'{path}/events.csv')
    t0 = [t for t, e, a in ev if e == 'PHASE:TRANSIT_TO_DROP_POINT'][0]
    t1 = [t for t, e, a in ev if e == 'PHASE:SETTLE_ABOVE_DROP_POINT']
    t2 = [t for t, e, a in ev if e == 'PHASE:DROP_OBJECT']
    return t0, (t1[0] if t1 else None), (t2[0] if t2 else None)


fig = plt.figure(figsize=(15, 6.0))
gs = fig.add_gridspec(1, 3, width_ratios=[1.15, 1.2, 1.0])
axA, axB, axC = fig.add_subplot(gs[0]), fig.add_subplot(gs[1]), fig.add_subplot(gs[2])
for lab, path, col in CASES:
    r = load_run(f'{path}/run.csv')
    t0, t1, t2 = transit(path)
    m = r['t'] >= t0
    tt = r['t'][m] - t0
    x, y, z = r['d3_x'][m], r['d3_y'][m], r['d3_z'][m]
    cx, cy = r['payload_x'][m], r['payload_y'][m]
    dist = np.hypot(x - cx, y - cy)
    end = (tt <= 60.0) & ((tt <= t2 - t0) if t2 else True)
    axB.plot(tt[end], dist[end], color=col, lw=2, label=lab)
    if t1 is not None:
        k = int(np.argmin(np.abs(tt - (t1 - t0))))
        axB.plot(tt[k], dist[k], 'o', color=col, mec='k', zorder=5)
        axB.annotate(f'SETTLE at +{t1 - t0:.1f} s', (tt[k], dist[k]), (tt[k] + 3, 0.30 + 0.08 * (col == '#5dade2')),
                     fontsize=8, color=col, arrowprops=dict(arrowstyle='->', color=col, lw=0.8))
        sel = tt <= (t1 - t0)
        axB.plot(t2 - t0, dist[end][-1], 'x', color=col, ms=8, mew=2)
    else:
        sel = np.ones_like(tt, bool)
        print(lab, 'final dist', dist[-1], 'z', z[-1], 'after', tt[-1], 's')
    axA.plot(x[sel], y[sel], color=col, lw=2, label=lab)
    axA.plot(x[sel][-1], y[sel][-1], 'o', color=col, mec='k', ms=8, zorder=5)
s = at(load_run(f'{R4374}/run.csv'), 54.83)
P = plates(s)
for i in range(3):
    axA.plot([P[i][0], s[f'd{i}_x']], [P[i][1], s[f'd{i}_y']], color=C_CARRIER, lw=1.5)
    axA.plot(s[f'd{i}_x'], s[f'd{i}_y'], 's', color=C_CARRIER, ms=7)
th = np.linspace(0, 2 * np.pi, 100)
axA.plot(PLATE_R * np.cos(th), PLATE_R * np.sin(th), 'k', lw=2)
axA.plot(0, 0, '*', color=C_DROP, ms=15, mec='k', zorder=6)
# where the legacy variants stood at their last replan
var = {'R4378': 'swing 3', 'R4379': 'inflation 0.04', 'R4380': 'carrier body 0.09', 'R4381': 'spline 6'}
mk = ['v', '^', '<', '>']
for (rid, txt), m_ in zip(var.items(), mk):
    f = glob.glob(f'{STALL}/{rid}_*/replans.csv')[0]
    rows = list(csv.DictReader(open(f)))
    q = [float(rows[-1][k]) for k in ('initial_q0_x', 'initial_q0_y', 'initial_q0_z')]
    axA.plot(q[0], q[1], m_, color='0.25', ms=7, label=f'{rid} legacy, {txt}: last replan start')
    print(rid, q, 'dist', np.hypot(q[0], q[1]))
axA.set_aspect('equal'); axA.set_xlim(-0.75, 0.75); axA.set_ylim(-0.3, 2.0)
axA.annotate('all five legacy runs stop\nat y 0.63-0.67 m', (0.03, 0.66), (0.12, 1.05), fontsize=8, color=C_LEG,
             arrowprops=dict(arrowstyle='->', color=C_LEG, lw=0.8))
axA.set_xlabel('x (m)'); axA.set_ylabel('y (m)')
axA.set_title('QUAD4 body path in transit (top view)')
axA.legend(fontsize=7, loc='upper center', bbox_to_anchor=(0.5, -0.14), ncol=1)
axB.axhline(0.12, color='k', ls=':', lw=1)
axB.text(41, 0.14, 'capture needs < 0.12 m', fontsize=8)
axB.text(26, 0.03, 'x = object dropped (curves end)', fontsize=8)
axB.set_xlabel('time since TRANSIT_TO_DROP_POINT (s)')
axB.set_ylabel('QUAD4 body to ring centre, horizontal (m)')
axB.set_xlim(0, 60); axB.set_ylim(0, 1.6)
axB.legend(loc='upper right')
axB.set_title('Distance to the drop point: legacy parks at 0.67 m for\nthe rest of the run (212 s); octagon arrives in ~10 s')
# R4374's statuses
cnt = collections.Counter(r['status'] for r in csv.DictReader(open(f'{R4374}/replans.csv')))
n = sum(cnt.values())
top = cnt.most_common(4)
labs = [k.replace('LOCAL_PLAN_FAILED:', 'FAILED:\n').replace('_PARTIAL_FINAL', '_PARTIAL\n_FINAL') for k, _ in top]
vals = [100 * v / n for _, v in top]
axC.barh(range(len(top))[::-1], vals, color=[C_LEG if 'CORRIDOR' in k else '0.6' for k, _ in top])
axC.set_yticks(range(len(top))[::-1]); axC.set_yticklabels(labs, fontsize=7.5)
for i, v in enumerate(vals):
    axC.text(v + 1, len(top) - 1 - i, f'{v:.0f} % ({top[i][1]})', va='center', fontsize=8)
axC.set_xlim(0, 100); axC.set_xlabel(f'share of R4374\'s {n} replans (%)')
axC.set_title('R4374: why each replan ended (his replans.csv,\ntop 4; the rest are 1 % together)')
fig.suptitle('Fig 4. Same hover, same settings: legacy_square stalls 0.67 m short for good (PREFIX_CORRIDOR_INCOMPATIBLE: '
             'the committed prefix sits on the hull edge); outer_octagon reaches the drop point.', fontsize=10.5)
fig.tight_layout(rect=(0, 0, 1, 0.94))
save(fig, 'fig4_legacy_stall_vs_octagon_reach.png')
