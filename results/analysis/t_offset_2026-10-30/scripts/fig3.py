import sys; sys.path.insert(0, '.')
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
OUT = '/home/wesley/multi_drone_control/results/analysis/t_offset_2026-10-30'
# settled tilt (deg), lateral integral |int_fxy| (N), group; values from the stage analysis (t_e+15..20 s; X1 t_e+12..18 s)
D = [
 ('R2823 release, int xy off', 0.60, 0.000, 'T, xy integral off'),
 ('R2312 soft detach', 0.69, 0.000, 'T, xy integral off'),
 ('R2821 soft detach', 0.69, 0.000, 'T, xy integral off'),
 ('R2822 soft detach', 0.69, 0.000, 'T, xy integral off'),
 ('R2457 soft detach', 0.62, 0.000, 'T, xy integral off'),
 ('R2840 soft det. 1.15 kg', 0.68, 0.000, 'T, xy integral off'),
 ('R2842 soft det. 1.15 kg', 0.67, 0.000, 'T, xy integral off'),
 ('R2834 1.15 kg, thr cap', 8.25, 0.000, 'T, xy off, throttle at its cap'),
 ('R2249 release 0.55 m', 6.61, 0.239, 'T, xy integral on, measured pull'),
 ('R2965 ours', 8.69, 0.285, 'T, xy integral on, measured pull'), ('R2972 ours', 10.10, 0.301, 'T, xy integral on, measured pull'),
 ('R2967 late 0.5', 8.57, 0.279, 'T, xy integral on, measured pull'), ('R2974 late 0.5', 8.56, 0.278, 'T, xy integral on, measured pull'),
 ('R2968 late 1', 9.64, 0.294, 'T, xy integral on, measured pull'), ('R2975 late 1', 12.22, 0.327, 'T, xy integral on, measured pull'),
 ('R2969 late 2', 6.91, 0.260, 'T, xy integral on, measured pull'), ('R2976 late 2', 6.91, 0.259, 'T, xy integral on, measured pull'),
 ('R2977 planned pull', 0.54, 0.145, 'T, xy integral on, planned pull'),
 ('R2966 no resize', 2.21, 0.297, 'square plan (no resize), xy on'), ('R2973 no resize', 2.20, 0.297, 'square plan (no resize), xy on'),
 ('R2766 8q n=7', 1.06, 0.219, '8-quad uneven (n=7, n=5), xy on'), ('R2766 8q n=5', 1.21, 0.240, '8-quad uneven (n=7, n=5), xy on'),
 ('R2766 8q n=6,4', 0.03, 0.001, 'even (8q n=6/4, triangle R2708)'), ('R2708 triangle', 0.03, 0.005, 'even (8q n=6/4, triangle R2708)'),
]
col = {'T, xy integral off': '#2980b9', 'T, xy off, throttle at its cap': '#95a5a6', 'T, xy integral on, measured pull': '#c0392b',
       'T, xy integral on, planned pull': '#8e44ad', 'square plan (no resize), xy on': '#2c3e50',
       '8-quad uneven (n=7, n=5), xy on': '#e67e22', 'even (8q n=6/4, triangle R2708)': '#27ae60'}
fig, ax = plt.subplots(1, 2, figsize=(15, 6))
y = np.arange(len(D))
ax[0].barh(y, [d[1] for d in D], color=[col[d[3]] for d in D])
ax[0].set_yticks(y); ax[0].set_yticklabels([d[0] for d in D], fontsize=8); ax[0].invert_yaxis()
ax[0].set_xlabel('settled ring tilt (deg)'); ax[0].grid(alpha=.3, axis='x')
ax[0].axvline(2.56, color='k', ls=':', lw=1); ax[0].text(2.6, len(D) - 0.5, 'rig T t3p_2 (22 Oct, xy off): 2.56 mean, 0.9 about the pair axis', fontsize=7)
for g, c in col.items():
    m = [d for d in D if d[3] == g]
    ax[1].scatter([d[2] for d in m], [d[1] for d in m], color=c, label=g, s=45)
ax[1].set_xlabel("planner's lateral integral force |int_f xy| (N)"); ax[1].set_ylabel('settled ring tilt (deg)'); ax[1].grid(alpha=.3)
ax[1].legend(fontsize=8, loc='upper left')
fig.suptitle('Every T flown in the twin: the T tilts only with the lateral integral on and the measured pull (and once at the throttle cap)')
fig.tight_layout(); fig.savefig(f'{OUT}/q3_T_history.png', dpi=110)
