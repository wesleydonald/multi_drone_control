"""Fig 6: an ESTIMATE (not his code) of the room at the drop point vs QUAD4's body height, against the pads his model
adds, for several swing bounds; his probe's verdicts on the same twin state (R4374 stall README, outer_octagon,
swing 5) are overlaid as the check. Real room = centre-line distance from QUAD4's hanging cable+object (body over the
ring centre, the R4374 cable vector) to the nearest carrier tether. Pads = swing reach at the object
(1.109 m x sin swing) + the object's half-diagonal (twin box 0.08 -> 0.113) + carrier tracking 0.06 + ego tracking 0.05."""
import numpy as np

from common import *

s = at(load_run(f'{R4374}/run.csv'), 54.83)
P = plates(s)
B = [np.array([s[f'd{i}_x'], s[f'd{i}_y'], s[f'd{i}_z']]) for i in range(3)]
L = s['d3_z'] - s['ball_z']          # body to object centre, 1.109 m
H = np.linspace(2.4, 3.4, 51)
room = []
for h in H:
    top = np.array([0, 0, h]); bot = top - np.array([0, 0, L])
    room.append(min(seg_dist(top, bot, P[i], B[i])[0] for i in range(3)))
room = np.array(room)
fixed = 0.08 * np.sqrt(2) + 0.06 + 0.05

fig, ax = plt.subplots(figsize=(10.5, 6))
ax.plot(H, room, color='k', lw=2.5, label='real room: QUAD4 cable/object to nearest carrier tether (centre lines)')
cols = {3: '#27ae60', 5: C_OCT, 8: '#d68910', 10: C_LEG}
for sw, col in cols.items():
    need = fixed + L * np.sin(np.radians(sw))
    ax.axhline(need, color=col, ls='--', lw=1.6)
    hx = np.interp(need, room, H)
    ax.plot(hx, need, 'o', color=col, mec='k', ms=7, zorder=5)
    ax.text(3.39, need + 0.006, f'pads at swing {sw} deg: {need:.2f} m -> estimated lowest free body z {hx:.2f} m',
            color=col, fontsize=8.5, ha='right')
# his probe on the same twin state (README rows at y = 0, x = 0; outer_octagon, swing 5)
for z, ok in [(2.50, False), (2.75, False), (2.90, True), (3.00, True)]:
    r = np.interp(z, H, room)
    ax.plot(z, r, 'v' if not ok else '^', color=C_OCT, ms=11, mec='k', zorder=6)
    off = {2.50: (2.42, 0.215), 2.75: (2.62, 0.47), 2.90: (2.93, 0.455), 3.00: (3.03, 0.505)}[z]
    ax.annotate(f'his probe, z {z:.2f}:\n' + ('free' if ok else 'BLOCKED'), (z, r), off, fontsize=8, color=C_OCT,
                arrowprops=dict(arrowstyle='->', color=C_OCT, lw=0.8))
ax.axvspan(2.75, 2.90, color=C_OCT, alpha=0.08)
ax.text(2.825, 0.56, 'his actual\nlowest free\nbody z at\nswing 5 is\nin this band', ha='center', fontsize=8, color=C_OCT)
ax.set_xlabel('QUAD4 body height over the drop point (m)')
ax.set_ylabel('horizontal clearance (m)')
ax.set_xlim(2.4, 3.4); ax.set_ylim(0.15, 0.75)
ax.legend(loc='upper left', fontsize=8.5)
ax.set_title('Fig 6. ESTIMATE: every degree of swing bound costs ~0.02 m of clearance = ~0.04 m of body height over the\n'
             'ring; at swing 5 his octagon hull blocks ~0.2-0.35 m higher than these pads alone predict (R4374 twin geometry)',
             fontsize=10.5)
fig.tight_layout()
save(fig, 'fig6_clearance_budget_vs_swing.png')
print('L', L, 'slope', np.polyfit(H, room, 1)[0])
