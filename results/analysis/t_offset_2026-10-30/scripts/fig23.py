import sys; sys.path.insert(0, '.')
from load import *
from table2 import rows
from stages import R as SR
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT = '/home/wesley/multi_drone_control/results/analysis/t_offset_2026-10-30'
C = {'ours': '#c0392b', 'd05': '#e67e22', 'd10': '#b7950b', 'd20': '#27ae60', 'nors': '#2c3e50', 'plan': '#8e44ad'}
LAB = {'ours': 'ours', 'd05': 'late 0.5 s', 'd10': 'late 1 s', 'd20': 'late 2 s', 'nors': 'no resize', 'plan': 'planned pull (R2977)'}
# --- Q2: time series tilt + int_fy, and the phase plot tilt vs int_fy
fig, ax = plt.subplots(1, 3, figsize=(17, 4.8))
g = np.arange(-1, 20, 0.05)
for arm in ('nors', 'ours', 'd05', 'd10', 'd20', 'plan'):
    for j, r in enumerate(RUNS[arm]):
        if r == 2970: continue
        x = Run(r); t = x.df.t.values - x.te
        tilt = np.interp(g, t, np.degrees(np.arcsin(np.clip(x.df.ny.values, -1, 1))))
        fy = np.interp(g, x.pl.t.values - x.te, pd.to_numeric(x.pl.int_fy, errors='coerce').values)
        lab = LAB[arm] if (j == 0 or r == 2977) else None
        ax[0].plot(g, tilt, color=C[arm], lw=1, label=lab)
        ax[1].plot(g, -fy, color=C[arm], lw=1, label=lab)
        if x.tres is not None:
            m = g > (x.tres - x.te) + 1.0
            ax[2].plot(-fy[m], tilt[m], color=C[arm], lw=0.8, label=lab)
        else:
            m = g > 1.5
            ax[2].plot(-fy[m], tilt[m], color=C[arm], lw=0.8, label=lab)
ax[0].set_ylim(-5, 20); ax[0].set_xlim(-1, 20); ax[0].set_xlabel('t - t_e (s)'); ax[0].set_ylabel('tilt about the 0-6 axis (deg)')
ax[1].set_xlim(-1, 20); ax[1].set_xlabel('t - t_e (s)'); ax[1].set_ylabel("planner's lateral integral force, -int_fy (N)")
ax[2].set_xlim(-0.02, 0.4); ax[2].set_ylim(-5, 20); ax[2].set_xlabel('-int_fy (N)'); ax[2].set_ylabel('tilt (deg)')
ax[2].set_title('after the resize + 1 s (no resize: after t_e + 1.5 s)')
for a in ax: a.grid(alpha=.3); a.legend(fontsize=8)
fig.suptitle('sqT: the tilt grows with the planner\'s lateral (xy) integral once the T is planned; the 4-cable plan and the planned pull do not tilt with it')
fig.tight_layout(); fig.savefig(f'{OUT}/q2_tilt_vs_lateral_integral.png', dpi=110)
# --- Q2: the twin's statics: tilt vs the inferred net lift at plate 3
fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
for rw in rows:
    ax[0].scatter(rw['ringz_from_rod3'], rw['tilt_x'], color=C[rw['arm']], s=40)
    ax[0].annotate(f"R{rw['run']}", (rw['ringz_from_rod3'], rw['tilt_x']), fontsize=7, xytext=(3, 3), textcoords='offset points')
    ax[1].scatter(-rw['d3'][1], rw['d3'][2], color=C[rw['arm']], s=40, label=rw['arm'])
ax[0].set_xlabel('inferred net upward pull on plate 3 (N)')
ax[0].set_ylabel('settled tilt (deg, t_e+12..18 s)'); ax[0].grid(alpha=.3)
h, l = ax[1].get_legend_handles_labels(); d = dict(zip(l, h)); ax[1].legend(d.values(), d.keys(), fontsize=8)
ax[1].set_xlabel('drone 3 (plate 3) distance out from the centre, -y (m)'); ax[1].set_ylabel('drone 3 height (m)'); ax[1].grid(alpha=.3)
ax[1].set_title('where the planner holds the plate-3 carrier')
fig.tight_layout(); fig.savefig(f'{OUT}/q2_plate3_lift_and_drone3.png', dpi=110)
