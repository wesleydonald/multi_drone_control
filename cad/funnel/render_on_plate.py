"""A 3D picture of a funnel variant bolted on its plate of the ring section, with QUAD4's holder seated in it: the whole
funnel, and the same view with the near half of the funnel cut away. Also writes the scene as a coloured .glb (opens in
any 3D viewer) in the ring section's frame.

    PYTHONPATH=~/venvs/cad_pkgs python3 -s cad/funnel/render_on_plate.py --variant v6 [--out cad/funnel/out]
"""
import argparse
import math
import os
import sys

import cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.collections import PolyCollection  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import assembly  # noqa: E402
import funnel  # noqa: E402

GREY, BLUE, RED, DARK = (0.62, 0.62, 0.62), (0.45, 0.65, 0.92), (0.84, 0.27, 0.27), (0.25, 0.25, 0.25)


def mesh(shape, tol=0.4):
    verts, tris = shape.tessellate(tol)
    return np.array([v.toTuple() for v in verts]), np.array(tris)


def scene(p, cut_near_half):
    """The parts in the ring section's frame (mm) as (vertices, triangles, colour)."""
    a = math.radians(assembly.PLATE_AZ_DEG)
    cx, cy = p['plate_r_ring'] * math.cos(a), p['plate_r_ring'] * math.sin(a)

    def place(s, z):
        return s.rotate((0, 0, 0), (0, 0, 1), assembly.PLATE_AZ_DEG).translate(cq.Vector(cx, cy, z))

    f = funnel.build(p).val()
    if cut_near_half:                                     # the anticlockwise half faces the camera
        f = f.cut(cq.Workplane('XY').box(400, 200, 300, centered=(True, False, True)).val())
    ring = assembly.stl_mm(assembly.RING)
    P, T = mesh(ring)
    keep = np.linalg.norm(P[T].mean(axis=1)[:, :2] - [cx, cy], axis=1) < 150.0   # the ring near the plate only
    parts = [(P, T[keep], GREY),
             (*mesh(place(f, assembly.TOP_Z)), BLUE),
             (*mesh(place(assembly.stl_mm(assembly.HOLDER), assembly.STEEL_Z + p['magnet_below_disc'])), RED),
             (*mesh(place(cq.Workplane('XY').circle(10.0).extrude(p['magnet_below_disc']).val(), assembly.STEEL_Z)), DARK)]
    return parts, np.array([cx, cy, 10.0])


def draw(ax, parts, centre, az_deg, el_deg):
    """Painter's algorithm: every triangle projected for a camera at (az, el) around the centre, drawn far to near."""
    az, el = math.radians(az_deg), math.radians(el_deg)
    d = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])   # towards the camera
    right = np.cross(-d, [0, 0, 1])
    right /= np.linalg.norm(right)
    up = np.cross(right, -d)
    light = d + 0.6 * right + 0.8 * np.array([0, 0, 1])
    light /= np.linalg.norm(light)
    polys, cols, depth = [], [], []
    for P, T, c in parts:
        tri = P[T] - centre
        n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
        s = 0.35 + 0.65 * np.abs(n @ light)
        polys.append(np.stack([tri @ right, tri @ up], axis=2))
        cols.append(np.clip(np.outer(s, c), 0, 1))
        depth.append((tri @ d).mean(axis=1))
    polys, cols, depth = np.concatenate(polys), np.concatenate(cols), np.concatenate(depth)
    order = np.argsort(depth)
    ax.add_collection(PolyCollection(polys[order], facecolors=cols[order], edgecolors=cols[order], linewidths=0.15))
    ax.set_xlim(-130, 130), ax.set_ylim(-75, 105), ax.set_aspect('equal'), ax.set_axis_off()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--variant', default='v6', choices=sorted(funnel.VARIANTS))
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out'))
    a = ap.parse_args()
    p = funnel.VARIANTS[a.variant]
    c = funnel.checks(p, funnel.build(p))
    view = (assembly.PLATE_AZ_DEG + 35.0, 32.0)           # from outside the ring, a little to the anticlockwise side
    fig, axes = plt.subplots(1, 2, figsize=(18, 7.5))
    for ax, cut, title in ((axes[0], False, 'on the plate'), (axes[1], True, 'near half cut away: the holder seated')):
        parts, centre = scene(p, cut)
        draw(ax, parts, centre, *view)
        ax.set_title(title, fontsize=12)
    fig.suptitle(f"Funnel {a.variant} on its plate (ring grey, funnel blue, QUAD4's holder red, magnet dark): "
                 f"{c['height_top_mm (above the ring top)']:.1f} mm tall, mouth {c['mouth_radial_mm (full)']:.0f} x "
                 f"{c['mouth_tangential_mm (full)']:.0f} mm, {c['mass_g']:.0f} g", fontsize=13)
    fig.tight_layout()
    png = os.path.join(a.out, f'funnel_{a.variant}_on_plate_3d.png')
    fig.savefig(png, dpi=110)
    print('wrote', png)

    pa = math.radians(assembly.PLATE_AZ_DEG)
    cx, cy = p['plate_r_ring'] * math.cos(pa), p['plate_r_ring'] * math.sin(pa)
    rot = cq.Location(cq.Vector(cx, cy, 0), cq.Vector(0, 0, 1), assembly.PLATE_AZ_DEG)
    assy = cq.Assembly(name=f'funnel_{a.variant}_on_plate')
    assy.add(assembly.stl_mm(assembly.RING), name='ring_section', color=cq.Color(*GREY))
    assy.add(funnel.build(p), name='funnel', color=cq.Color(*BLUE),
             loc=cq.Location(cq.Vector(cx, cy, assembly.TOP_Z)) * cq.Location(cq.Vector(), cq.Vector(0, 0, 1),
                                                                               assembly.PLATE_AZ_DEG))
    assy.add(assembly.stl_mm(assembly.HOLDER), name='magnet_holder_seated', color=cq.Color(*RED),
             loc=cq.Location(cq.Vector(cx, cy, assembly.STEEL_Z + p['magnet_below_disc'])) *
             cq.Location(cq.Vector(), cq.Vector(0, 0, 1), assembly.PLATE_AZ_DEG))
    assy.add(cq.Workplane('XY').circle(10.0).extrude(p['magnet_below_disc']), name='electromagnet_20x20',
             color=cq.Color(*DARK), loc=rot)
    glb = os.path.join(a.out, f'funnel_{a.variant}_on_plate.glb')
    assy.save(glb)
    print('wrote', glb, f'({os.path.getsize(glb) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
