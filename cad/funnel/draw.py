"""A one-page drawing of a funnel variant: a cut along the ring radius, a cut through an outer bolt, the plan 2 mm up, and a
view from above. The seated magnet holder (disc and magnet) is drawn in red in the two cuts.

    PYTHONPATH=~/venvs/cad_pkgs python3 -s cad/funnel/draw.py --variant v5 [--out cad/funnel/out]
"""
import argparse
import os
import sys

import cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import funnel  # noqa: E402

BLUE, RED, PINK = '#2b6cb0', '#d64545', '#f2a7a7'


def _section_polys(shape, origin, normal, axes):  # the plan view
    """Triangles of the solid's cut by the plane (origin, normal), in the two plot axes given as (i, j) indices."""
    cut = shape.intersect(cq.Face.makePlane(500, 500, basePnt=origin, dir=normal))
    polys = []
    for face in cut.Faces():
        verts, tris = face.tessellate(0.05)
        for t in tris:
            polys.append([(verts[k].toTuple()[axes[0]], verts[k].toTuple()[axes[1]]) for k in t])
    return polys


def _holder(ax, p):
    """The seated holder in a vertical cut: the disc (red) and the magnet under it (pink), z from the ring top."""
    disc_bottom = -p['steel_below_top'] + p['magnet_below_disc']
    ax.add_patch(plt.Rectangle((-p['disc_d'] / 2, disc_bottom), p['disc_d'], p['disc_t'], color=RED))
    ax.add_patch(plt.Rectangle((-10.0, -p['steel_below_top']), 20.0, p['magnet_below_disc'], color=PINK))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variant', default='v5', choices=sorted(funnel.VARIANTS))
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out'))
    a = ap.parse_args()
    p = funnel.VARIANTS[a.variant]
    part = funnel.build(p)
    solid = part.val()
    c = funnel.checks(p, part)
    bolt = [h for h in funnel.holes(p) if h[0] == p['bolts'][0]][0]

    fig = plt.figure(figsize=(22, 7))
    fig.suptitle(f"Funnel {a.variant} (TPU): {2 * p['bore_r']:.0f} mm base, {c['height_top_mm (above the ring top)']:.0f} mm "
                 f"tall, bore {p['bore_top']:.0f} mm, {c['mouth_radial_mm (full)']:.0f} x {c['mouth_tangential_mm (full)']:.0f} mm "
                 f"mouth, {p['cone_angle_radial_deg']:.0f} deg wall, magnet within {c['disc_play_mm (magnet within this of the centre)']:.1f} mm "
                 f"of the centre; {c['mass_g']:.0f} g", fontsize=13)
    import math
    d = (bolt[1] / math.hypot(bolt[1], bolt[2]), bolt[2] / math.hypot(bolt[1], bolt[2]))
    for k, (title, normal, along) in enumerate((
            ('cut along the ring radius (through the in and out ribs)', (0, 1, 0), (1.0, 0.0)),
            (f"cut through the plate centre and the outer bolt ({bolt[0]}) with its nut", (-d[1], d[0], 0), d))):
        ax = fig.add_subplot(1, 4, k + 1)
        cut = solid.intersect(cq.Face.makePlane(500, 500, basePnt=(0, 0, 0), dir=normal))
        polys = []
        for face in cut.Faces():
            verts, tris = face.tessellate(0.05)
            pts = [v.toTuple() for v in verts]
            polys += [[(pts[i][0] * along[0] + pts[i][1] * along[1], pts[i][2]) for i in t] for t in tris]
        ax.add_collection(PolyCollection(polys, facecolor=BLUE, edgecolor=BLUE, linewidth=0.3))
        _holder(ax, p)
        ax.axhline(0, color='0.3', lw=0.8)
        ax.set_xlim(-110, 110), ax.set_ylim(-8, 95), ax.set_aspect('equal')
        ax.set_xlabel('from the plate centre (mm)'), ax.set_ylabel('above the ring top (mm)')
        ax.set_title(title, fontsize=10), ax.grid(alpha=0.3)
    ax = fig.add_subplot(1, 4, 3)
    ax.add_collection(PolyCollection(_section_polys(solid, (0, 0, 2.0), (0, 0, 1), (0, 1)), facecolor=BLUE,
                                     edgecolor=BLUE, linewidth=0.3))
    for name, x, y in funnel.holes(p):
        used = name in p['bolts']
        ax.plot(x, y, 'o', ms=10, mfc='white', mec='k')
        ax.annotate('bolt' if used else ('zip tie: notched' if name == 'in_cw' else 'unused'), (x, y),
                    xytext=(5, 6), textcoords='offset points', fontsize=8)
    ax.add_patch(plt.Circle((0, 0), p['disc_d'] / 2, fill=False, ls='--', color=RED))
    ax.text(0, 2, f"disc {p['disc_d']:.0f} mm", color=RED, fontsize=8, ha='center')
    ax.set_xlim(-65, 65), ax.set_ylim(-65, 65), ax.set_aspect('equal'), ax.grid(alpha=0.3)
    ax.set_xlabel('radial (mm)'), ax.set_ylabel('along the rim (mm)')
    ax.set_title('plan at 2 mm up: the foot, the bolts, the zip-tie notch', fontsize=10)
    # the view from above: the tessellated solid turned to elev / azim, drawn back to front and shaded by its normals
    # (no mplot3d: the CAD environment's matplotlib and the system's mpl_toolkits do not match)
    ax = fig.add_subplot(1, 4, 4)
    verts, tris = solid.tessellate(0.4)
    P = np.array([v.toTuple() for v in verts])
    T = np.array(tris)
    el, az = np.radians(30.0), np.radians(-60.0)
    Rz = np.array([[np.cos(az), -np.sin(az), 0], [np.sin(az), np.cos(az), 0], [0, 0, 1]])
    R = P @ Rz.T                                         # turned about the vertical
    Q = np.stack([R[:, 0],                               # screen x
                  R[:, 2] * np.cos(el) + R[:, 1] * np.sin(el),   # screen y: up
                  -R[:, 1] * np.cos(el) + R[:, 2] * np.sin(el)], axis=1)  # towards the viewer
    tri = Q[T]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = np.array([0.3, 0.6, 0.75]) / np.linalg.norm([0.3, 0.6, 0.75])
    shade = 0.45 + 0.55 * np.abs(n @ light)
    order = np.argsort(tri[:, :, 2].mean(axis=1))
    cols = [(0.61 * s_, 0.76 * s_, 0.92 * s_) for s_ in np.clip(shade[order], 0, 1)]
    ax.add_collection(PolyCollection(tri[order][:, :, [0, 1]], facecolors=cols, edgecolors='none'))
    ax.set_xlim(Q[:, 0].min() - 5, Q[:, 0].max() + 5), ax.set_ylim(Q[:, 1].min() - 5, Q[:, 1].max() + 5)
    ax.set_aspect('equal'), ax.set_axis_off(), ax.set_title('from above', fontsize=10)
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, f"funnel_{a.variant}_{2 * p['bore_r']:.0f}mm_tpu.png")
    fig.savefig(path, dpi=110, bbox_inches='tight')
    print('wrote', path)


if __name__ == '__main__':
    main()
