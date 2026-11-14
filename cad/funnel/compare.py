"""Funnel variants side by side: the cut along the ring radius (with the seated holder) and the view from above, at one
scale, with each variant's height, mouth, centring play and mass.

    PYTHONPATH=~/venvs/cad_pkgs python3 -s cad/funnel/compare.py v4 v5 v6 [--out cad/funnel/out]
"""
import argparse
import os
import sys

import cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import draw  # noqa: E402
import funnel  # noqa: E402


def radial_cut(solid):
    """Triangles of the cut by the plane through the plate centre along the ring radius, as (radial, height)."""
    cut = solid.intersect(cq.Face.makePlane(500, 500, basePnt=(0, 0, 0), dir=(0, 1, 0)))
    polys = []
    for face in cut.Faces():
        verts, tris = face.tessellate(0.05)
        pts = [v.toTuple() for v in verts]
        polys += [[(pts[i][0], pts[i][2]) for i in t] for t in tris]
    return polys


def view(ax, solid, el=30.0, az=-60.0):
    """The solid from above, drawn back to front and shaded by its normals."""
    verts, tris = solid.tessellate(0.4)
    P, T = np.array([v.toTuple() for v in verts]), np.array(tris)
    el, az = np.radians(el), np.radians(az)
    R = P @ np.array([[np.cos(az), -np.sin(az), 0], [np.sin(az), np.cos(az), 0], [0, 0, 1]]).T
    Q = np.stack([R[:, 0], R[:, 2] * np.cos(el) + R[:, 1] * np.sin(el), -R[:, 1] * np.cos(el) + R[:, 2] * np.sin(el)], axis=1)
    tri = Q[T]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    light = np.array([0.3, 0.6, 0.75]) / np.linalg.norm([0.3, 0.6, 0.75])
    shade = np.clip(0.45 + 0.55 * np.abs(n @ light), 0, 1)
    order = np.argsort(tri[:, :, 2].mean(axis=1))
    ax.add_collection(PolyCollection(tri[order][:, :, [0, 1]], facecolors=[(0.61 * s, 0.76 * s, 0.92 * s) for s in shade[order]],
                                     edgecolors='none'))
    return Q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('variants', nargs='+', choices=sorted(funnel.VARIANTS))
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out'))
    a = ap.parse_args()
    k = len(a.variants)
    fig, axes = plt.subplots(2, k, figsize=(6.2 * k, 9.5), gridspec_kw={'height_ratios': [1, 1.15]})
    views = []
    for j, v in enumerate(a.variants):
        p = funnel.VARIANTS[v]
        part = funnel.build(p)
        c = funnel.checks(p, part)
        ax = axes[0, j]
        ax.add_collection(PolyCollection(radial_cut(part.val()), facecolor=draw.BLUE, edgecolor=draw.BLUE, linewidth=0.3))
        draw._holder(ax, p)
        ax.axhline(0, color='0.3', lw=0.8)
        ax.set_xlim(-110, 110), ax.set_ylim(-8, 95), ax.set_aspect('equal'), ax.grid(alpha=0.3)
        ax.set_xlabel('from the plate centre, along the ring radius (mm)')
        if j == 0:
            ax.set_ylabel('above the ring top (mm)')
        ax.set_title(f"{v}: {c['height_top_mm (above the ring top)']:.1f} mm tall, mouth {c['mouth_radial_mm (full)']:.0f} x "
                     f"{c['mouth_tangential_mm (full)']:.0f} mm\nbore {p['bore_top']:.0f} mm, magnet within "
                     f"{c['disc_play_mm (magnet within this of the centre)']:.1f} mm of the centre, {c['mass_g']:.0f} g (TPU)",
                     fontsize=11)
        views.append((axes[1, j], view(axes[1, j], part.val())))
    lo = np.min([Q[:, :2].min(axis=0) for _, Q in views], axis=0) - 5
    hi = np.max([Q[:, :2].max(axis=0) for _, Q in views], axis=0) + 5
    for ax, _ in views:                                   # the views at one scale
        ax.set_xlim(lo[0], hi[0]), ax.set_ylim(lo[1], hi[1]), ax.set_aspect('equal'), ax.set_axis_off()
    fig.suptitle('Funnel variants at one scale (the red disc and pink magnet: QUAD4\'s holder seated on the plate)', fontsize=13)
    fig.tight_layout()
    path = os.path.join(a.out, 'funnel_compare_' + '_'.join(a.variants) + '.png')
    fig.savefig(path, dpi=110)
    print('wrote', path)


if __name__ == '__main__':
    main()
