#!/usr/bin/env python3
"""seat_gap_vs_outcome.png from tries.csv: (a) the seat gap of every lift try by object, held or not;
(b) the same tries, seat gap against the magnet's tilt on the object."""
import os
import numpy as np, pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
df = pd.read_csv(os.path.join(HERE, 'tries.csv'))
lift = df[df.lifted].copy()
lift['ok'] = lift.outcome.str.startswith('held')
BLUE, RED = '#1f5fbf', '#c8312b'
order = ['small object + funnel', 'small object, bare (54 mm disc, 60 g)', 'big object 100x100x10 mm (0.15 kg)',
         'puck (his commissioning puck)']
short = {order[0]: 'small object\n+ funnel (11-13 Nov)', order[1]: 'small object,\nbare (5-11 Nov)',
         order[2]: 'big object\n(4-11 Nov)', order[3]: 'puck (14 Oct,\nold tip body)'}
rng = np.random.default_rng(3)

fig, (a, b) = plt.subplots(1, 2, figsize=(10.5, 4.6), gridspec_kw={'width_ratios': [1.15, 1]})
for i, obj in enumerate(order):
    g = lift[lift.object == obj]
    y = len(order) - 1 - i + rng.uniform(-0.18, 0.18, len(g))
    for ok, c, m, lab in ((True, BLUE, 'o', 'held (object rose with the tip)'), (False, RED, 'X', 'not held')):
        s = g.ok.values == ok
        a.scatter(g.seat_gap_mm[s], y[s], c=c, marker=m, s=46, edgecolors='white', linewidths=0.6, zorder=3,
                  label=lab if i == 0 else None)
a.axvline(0, color='0.35', lw=1, ls='--', zorder=1)
XMAX = 7.5
a.set_xlim(-2.5, XMAX)
a.set_yticks(range(len(order)))
a.set_yticklabels([short[o] for o in order[::-1]], fontsize=8.5)
a.set_xlabel('seat gap before the lift (mm): tip over object - contact height')
a.set_title('(a) seat gap by object', fontsize=10, loc='left')
a.grid(axis='x', color='0.9', lw=0.6)

for ok, c, m, lab in ((True, BLUE, 'o', 'held'), (False, RED, 'X', 'not held')):
    g = lift[lift.ok == ok]
    b.scatter(g.seat_gap_mm, g.seat_tilt_deg, c=c, marker=m, s=46, edgecolors='white', linewidths=0.6, zorder=3,
              label=lab)
b.axvline(0, color='0.35', lw=1, ls='--', zorder=1)
b.set_xlabel('seat gap before the lift (mm)')
b.set_ylabel('magnet tilt on the object (deg)')
b.set_title('(b) seat gap and magnet tilt', fontsize=10, loc='left')
b.set_xlim(-2.5, XMAX)
unc = lift[~lift.tilt_calibrated]
b.scatter(unc.seat_gap_mm.clip(upper=XMAX - 0.25), unc.seat_tilt_deg, facecolors='none', edgecolors='0.3', s=110,
          lw=0.8, zorder=2, label='14 Oct (old tip body: tilt not calibrated)')
# points beyond the axis: drawn at its edge with their value
for ax, yv in ((a, None), (b, 'seat_tilt_deg')):
    for _, r in lift[lift.seat_gap_mm > XMAX].iterrows():
        y = (len(order) - 1 - order.index(r.object)) if yv is None else r[yv]
        ax.scatter([XMAX - 0.25], [y], c=RED, marker='>', s=40, zorder=4)
        ax.annotate(f'{r.seat_gap_mm:.1f} mm', (XMAX - 0.25, y), xytext=(-12, 0), textcoords='offset points',
                    ha='right', va='center', fontsize=8, color='0.25')
b.grid(color='0.9', lw=0.6)
for ax in (a, b):
    for s in ax.spines.values():
        s.set_visible(True)
h, l = a.get_legend_handles_labels()
hb, lb = b.get_legend_handles_labels()
h, l = h + hb[2:], l + lb[2:]
fig.legend(h, l, loc='lower center', ncol=3, frameon=True, fontsize=9, bbox_to_anchor=(0.5, 0.0))
fig.tight_layout(rect=(0, 0.07, 1, 1))
fig.savefig(os.path.join(HERE, 'seat_gap_vs_outcome.png'), dpi=160)
print('wrote seat_gap_vs_outcome.png', len(lift), 'lift tries')
