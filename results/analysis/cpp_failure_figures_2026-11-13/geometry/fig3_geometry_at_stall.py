"""Fig 3: the real geometry at R4374's stall (run.csv t 54.83 s): 3D view, top view with each tether's axis-aligned
box (what legacy_square uses for a slanted tether), and the real centre-line distances from QUAD4's cable/object to
the carrier tethers, at the stall and if QUAD4 were over the drop point."""
import numpy as np
from matplotlib.patches import Rectangle, Circle

from common import *

run = load_run(f'{R4374}/run.csv')
s = at(run, 54.83)
P = plates(s)
B = [np.array([s[f'd{i}_x'], s[f'd{i}_y'], s[f'd{i}_z']]) for i in range(3)]
q4 = np.array([s['d3_x'], s['d3_y'], s['d3_z']])
ball = np.array([s['ball_x'], s['ball_y'], s['ball_z']])
cab = ball - q4
q4d = np.array([0.0, 0.0, 2.90])               # QUAD4 body over the drop point, same hanging cable
balld = q4d + cab

fig = plt.figure(figsize=(15.5, 5.8))
ax3 = fig.add_subplot(1, 3, 1, projection='3d')
th = np.linspace(0, 2 * np.pi, 100)
c = np.array([s['payload_x'], s['payload_y'], s['payload_z']])
ax3.plot(c[0] + PLATE_R * np.cos(th), c[1] + PLATE_R * np.sin(th), c[2] + 0 * th, 'k', lw=2.5)
for i in range(3):
    ax3.plot(*zip(P[i], B[i]), color=C_CARRIER, lw=2)
    ax3.scatter(*B[i], color=C_CARRIER, s=50, marker='s')
    ax3.text(B[i][0], B[i][1], B[i][2] + 0.08, f'drone_{i}', color=C_CARRIER, fontsize=8)
ax3.plot(*zip(q4, ball), color=C_Q4, lw=2)
ax3.scatter(*q4, color=C_Q4, s=60, edgecolor='k')
ax3.scatter(*ball, color=C_Q4, s=40, edgecolor='k')
ax3.plot(*zip(q4d, balld), color=C_DROP, lw=1.2, ls=':')
ax3.scatter(*q4d, color=C_DROP, s=120, marker='*', edgecolor='k')
ax3.text(q4[0], q4[1] + 0.05, q4[2] + 0.08, 'QUAD4 (stalled)', color=C_Q4, fontsize=8)
ax3.set_xlabel('x (m)'); ax3.set_ylabel('y (m)'); ax3.set_zlabel('z (m)')
ax3.set_xlim(-0.8, 0.8); ax3.set_ylim(-0.8, 0.8); ax3.set_zlim(1.0, 3.0)
ax3.view_init(elev=18, azim=-60)
ax3.set_title('3D: ring, three carriers on slanted 1 m tethers,\nQUAD4 at the stall (dotted: over the drop point)')

# top view with the tethers' axis-aligned boxes
ax = fig.add_subplot(1, 3, 2)
for i in range(3):
    lo, hi = np.minimum(P[i], B[i]), np.maximum(P[i], B[i])
    ax.add_patch(Rectangle(lo[:2], *(hi - lo)[:2], fc=C_CARRIER, alpha=0.12, ec=C_CARRIER, lw=1, ls='--'))
    ax.plot(*zip(P[i][:2], B[i][:2]), color=C_CARRIER, lw=2)
    ax.plot(*B[i][:2], 's', color=C_CARRIER, ms=8)
    ax.text(B[i][0], B[i][1] + (0.07 if i else -0.12), f'drone_{i}\nbox {hi[0]-lo[0]:.2f} x {hi[1]-lo[1]:.2f} x '
            f'{hi[2]-lo[2]:.2f} m', color=C_CARRIER, fontsize=7.5, ha='center')
ax.add_patch(Circle(c[:2], PLATE_R, fc='none', ec='k', lw=2.5))
ax.text(0.0, -0.32, 'plates r 0.225', ha='center', fontsize=7.5)
ax.plot(*q4[:2], 'o', color=C_Q4, ms=10, mec='k')
ax.plot(0, 0, '*', color=C_DROP, ms=15, mec='k')
ax.annotate('', (0, 0.03), (q4[0], q4[1] - 0.03), arrowprops=dict(arrowstyle='->', color=C_Q4, lw=1.2, ls='--'))
ax.text(0.03, 0.47, '0.68 m left\nto the drop point', color=C_Q4, fontsize=8)
ax.text(-0.95, -1.08, 'dashed boxes: each tether\'s axis-aligned box (the\nshape legacy_square inflates); the\n'
        'tether itself is the thin line inside it', fontsize=7.5)
ax.set_aspect('equal'); ax.set_xlim(-1.0, 1.0); ax.set_ylim(-1.1, 1.0)
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.set_title('Top view: a slanted tether becomes a\n0.44 x 0.25 x 0.93 m box in legacy mode')

# real centre-line distances from QUAD4's cable+object to each tether, vs height of the cable point
ax = fig.add_subplot(1, 3, 3)
zz = np.linspace(ball[2], q4[2], 120)
for lab, top, col in [('QUAD4 at the stall (y 0.68)', q4, C_Q4), ('QUAD4 over the drop point (z 2.90)', q4d, C_DROP)]:
    bot = top + cab
    dmin = []
    for z in zz:
        f = (z - bot[2]) / (top[2] - bot[2])
        p = bot + f * (top - bot)
        dmin.append(min(seg_dist(p, p, P[i], B[i])[0] for i in range(3)))
    ax.plot(dmin, zz, color=col, lw=2, label=lab)
    i_min = int(np.argmin(dmin))
    ax.plot(dmin[i_min], zz[i_min], 'o', color=col, mec='k')
    ax.annotate(f'{dmin[i_min]:.2f} m at z {zz[i_min]:.2f}', (dmin[i_min], zz[i_min]),
                (dmin[i_min] - 0.42, zz[i_min] + (0.33 if col == C_DROP else 0.18)), fontsize=8.5, color=col,
                arrowprops=dict(arrowstyle='->', color=col, lw=0.8))
    print(lab, 'min', dmin[i_min], 'at', zz[i_min])
ax.axhspan(ball[2] - 0.08, ball[2] + 0.08, color=C_Q4, alpha=0.1)
ax.text(0.02, ball[2] + 0.1, 'object (twin box 0.08 half)', fontsize=8, color=C_Q4)
# separation of the two side tethers at the object height
for zc in (ball[2],):
    pts = []
    for i in (1, 2):
        f = (zc - P[i][2]) / (B[i][2] - P[i][2])
        pts.append(P[i] + f * (B[i] - P[i]))
    sep = np.linalg.norm(pts[0][:2] - pts[1][:2])
ax.text(0.02, 2.75, f'drone_1 and drone_2 tethers are\n{sep:.2f} m apart (centre lines) at the\n'
        f'object height z {ball[2]:.2f}', fontsize=8)
ax.set_xlim(0, 1.0); ax.set_ylim(1.6, 3.0)
ax.set_xlabel('distance to the nearest carrier tether centre line (m)')
ax.set_ylabel('height of the point on QUAD4\'s cable / object (m)')
ax.legend(loc='upper right', bbox_to_anchor=(1.0, 0.82))
ax.set_title('Real room: nearest carrier tether along\nQUAD4\'s hanging cable (centre lines, no pads)')
fig.suptitle('Fig 3. The geometry at R4374\'s stall (twin, t 54.83 s): the physical gap between the carrier tethers is '
             'wide; the boxes around them are what close it.', fontsize=10.5)
fig.tight_layout(rect=(0, 0, 1, 0.95), w_pad=3.0)
save(fig, 'fig3_geometry_at_stall_R4374.png')
print('tether separation at object height', sep)
for i in range(3):
    lo, hi = np.minimum(P[i], B[i]), np.maximum(P[i], B[i])
    print(i, 'box', hi - lo, 'lo', lo, 'hi', hi)
