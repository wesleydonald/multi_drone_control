"""Fig 2a: his outer_octagon free-space maps (top view, ring-centred, QUAD4 body-centre at fixed heights) on the real
12 Nov state fm9hc_4 at the settle instant, static vs the carriers' hulls swept over 1.0 s of the ring's motion.
Fig 2b: over all four 12 Nov flights, how the drop point's pocket closes as the sweep time grows (out/ and maps/)."""
import glob

import numpy as np
from matplotlib.colors import ListedColormap
from scipy import ndimage

from common import *

FL = 'fm9hc_4'
STEP, HALF = 0.02, 1.2


def load_map(fl, d, z):
    txt = open(f'{RT}/maps/{fl}_octagon_{d}_{z}.txt').read().splitlines()
    hdr = [l for l in txt if l.startswith('# t')][0].split()
    g = np.array([[c == '#' for c in l] for l in txt if l and l[0] in '#.' and not l.startswith('# t')])
    return g, hdr


def line(fl, d):
    v = open(f'{RT}/maps/{fl}_{d}_line.txt').read().split()
    f = lambda a, b: np.array([float(x) for x in v[a:b]])
    return dict(t=float(v[0]), ego=f(2, 5), ring=f(32, 35), vel=f(39, 42), bodies=f(42, 51).reshape(3, 3))


def settle_z(fl):
    zs = sorted(glob.glob(f'{RT}/maps/{fl}_octagon_0_*.txt'))
    return [z.rsplit('_', 1)[1][:-4] for z in zs]


cmap = ListedColormap(['white', C_BLOCK])
zs = settle_z(FL)
fig, axs = plt.subplots(2, 3, figsize=(13.5, 9.2))
for row, d in enumerate(('0', '1')):
    L = line(FL, d)
    for col, z in enumerate(zs):
        ax = axs[row, col]
        g, hdr = load_map(FL, d, z)
        ax.imshow(g, cmap=cmap, extent=[-HALF - STEP / 2, HALF + STEP / 2, -HALF - STEP / 2, HALF + STEP / 2],
                  origin='upper', interpolation='nearest', alpha=0.85)
        n = g.shape[0]; ci = n // 2
        clear = ndimage.distance_transform_edt(~g) * STEP
        th = np.linspace(0, 2 * np.pi, 100)
        ax.plot(0.225 * np.cos(th), 0.225 * np.sin(th), 'k', lw=1.8)
        for b in L['bodies']:
            rel = b[:2] - L['ring'][:2]
            ax.plot(*rel, 's', color=C_CARRIER, ms=8)
            u = rel / np.linalg.norm(rel)
            ax.plot([0.225 * u[0], rel[0]], [0.225 * u[1], rel[1]], color=C_CARRIER, lw=1.5)
        e = L['ego'][:2] - L['ring'][:2]
        ax.plot(*e, 'o', color=C_Q4, mec='k', ms=8)
        ax.plot(0, 0, '*', color=C_DROP, mec='k', ms=14)
        if g[ci, ci]:
            msg = 'drop point INSIDE'
        else:
            msg = f'pocket radius {clear[ci, ci]:.2f} m'
            ax.add_patch(plt.Circle((0, 0), clear[ci, ci], fc='none', ec=C_DROP, lw=1.5, ls='--'))
        if d == '1':
            v = L['vel'][:2]
            ax.annotate('', (v[0] * 1.0 * 4, v[1] * 1.0 * 4), (0, 0),
                        arrowprops=dict(arrowstyle='->', lw=2, color='k'))
        ax.set_title(f'body z {z} m, {"static" if d == "0" else "swept 1.0 s"}: {msg}', fontsize=9.5)
        ax.set_xlim(-1.2, 1.2); ax.set_ylim(-1.2, 1.2); ax.set_aspect('equal')
        if row == 1: ax.set_xlabel('x from ring centre (m)')
        if col == 0: ax.set_ylabel('y from ring centre (m)')
spd = np.hypot(*line(FL, '1')['vel'][:2])
axs[1, 0].text(-1.15, -1.15, f'arrow: ring velocity x 4 s ({spd:.2f} m/s,\n{spd:.2f} m of travel in the 1 s sweep)',
               fontsize=8, va='bottom', bbox=dict(fc='white', ec='0.7'))
axs[0, 0].text(-1.15, -1.15, 'red: QUAD4 body centre here collides\n(his outer_octagon hulls, swing 5 deg,\n'
               'carrier tracking 0.06)', fontsize=8, va='bottom', bbox=dict(fc='white', ec='0.7'))
fig.suptitle(f'Fig 2a. Real 12 Nov state ({FL}, settle instant t {line(FL, "0")["t"]:.1f} s, ring circling at '
             f'{spd:.2f} m/s): the drop point\'s free pocket (star, dashed circle) closes when the carriers\' hulls are '
             f'swept over 1 s.\nOrange dot = QUAD4, blue = carriers + tethers (projected), black = plate circle.',
             fontsize=10.5)
fig.tight_layout(rect=(0, 0, 1, 0.95))
save(fig, 'fig2a_octagon_maps_static_vs_swept.png')

# ---- 2b: metrics vs sweep, all flights
flights = ['fm9_5', 'fm9hc_2', 'fm9hc_4', 'fm9hc_5']
cols = {'fm9_5': '#7f8c8d', 'fm9hc_2': '#2471a3', 'fm9hc_4': '#c0392b', 'fm9hc_5': '#d68910'}
A = ('TRANSIT_TO_DROP_POINT', 'SETTLE_ABOVE_DROP_POINT')
sw = ['0', '0.5', '1', '1.5']
fig, axs = plt.subplots(1, 3, figsize=(15, 4.9))
for fl in flights:
    lab = fl + (' (ring still)' if fl == 'fm9_5' else ' (ring circling)')
    pin, floor = [], []
    for d in sw:
        rows = [l.split() for l in open(f'{RT}/out/{fl}_octagon_{d}.txt')][1:]
        a = [r for r in rows if r[1] in A]
        pin.append(100 * np.mean([r[4] == '1' for r in a]))
        floor.append(np.median([float(r[8]) for r in a]))
    xs = [float(x) for x in sw]
    axs[0].plot(xs, pin, 'o-', color=cols[fl], lw=2, label=lab)
    axs[1].plot(xs, floor, 'o-', color=cols[fl], lw=2, label=lab)
    pk = []
    for d in ('0', '1'):
        for z in ('2.90',):
            g, _ = load_map(fl, d, z)
            ci = g.shape[0] // 2
            pk.append(0.0 if g[ci, ci] else ndimage.distance_transform_edt(~g)[ci, ci] * STEP)
    pk2 = []
    for d in ('0', '1'):
        g, _ = load_map(fl, d, '2.76')
        ci = g.shape[0] // 2
        pk2.append(0.0 if g[ci, ci] else ndimage.distance_transform_edt(~g)[ci, ci] * STEP)
    axs[2].plot([0, 1], pk, 'o-', color=cols[fl], lw=2, label=f'{fl}, body z 2.90')
    axs[2].plot([0, 1], pk2, 's--', color=cols[fl], lw=1.5, label=f'{fl}, body z 2.76')
    print(fl, 'drop inside %', pin, 'floor', floor, 'pocket 2.90', pk, 'pocket 2.76', pk2)
axs[0].axhline(100, color=C_LEG, ls=':', lw=1.5)
axs[0].text(0.02, 93, 'legacy_square: 100 % at every sweep, every flight', color=C_LEG, fontsize=8)
axs[0].set_ylabel('transit + settle samples with the drop point\ninside his obstacle set (%)')
axs[0].set_title('Drop point blocked (outer_octagon)')
axs[1].axhline(3.20, color=C_LEG, ls=':', lw=1.5)
axs[1].text(0.02, 3.17, 'legacy_square: >= 3.20 everywhere (blocked\nup to the top of the probe\'s scan)', color=C_LEG,
            fontsize=8, va='top')
axs[1].set_ylabel('lowest body height over the drop point\nfree all the way up to 3.2 m, median (m)')
axs[1].set_title('How high QUAD4 must be to hang over the drop point')
axs[1].set_ylim(2.5, 3.25)
axs[2].set_ylabel('free pocket radius around the drop point (m)')
axs[2].set_title('Pocket at the settle instant (maps/, 0 = inside)')
axs[2].set_xticks([0, 1])
for ax in axs[:2]:
    ax.set_xticks(xs)
for ax in axs:
    ax.set_xlabel('sweep time of the moving carriers\' hulls (s)')
axs[0].legend(loc='upper left', bbox_to_anchor=(0.0, 0.88), fontsize=8)
axs[2].legend(fontsize=7, ncol=2, loc='upper right')
axs[2].set_ylim(0, 0.24)
fig.suptitle('Fig 2b. Real 12 Nov states, his outer_octagon: the longer the carriers\' hulls are swept, the more often '
             'the drop point is blocked and the higher QUAD4 must fly (swing 5 deg).', fontsize=10.5)
fig.tight_layout(rect=(0, 0, 1, 0.93))
save(fig, 'fig2b_pocket_metrics_vs_sweep.png')
