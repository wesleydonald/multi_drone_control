"""An assembly for checking the funnel in Onshape: the ring section (Wesley's Ring_Section.stl), the funnel bolted on
the plate at 60 deg (both skins are whole there), and QUAD4's magnet holder seated in it (its disc 20 mm above the steel).
The STL parts come in as faceted bodies (mm); the funnel is the solid from funnel.py.

    PYTHONPATH=~/venvs/cad_pkgs python3 -s cad/funnel/assembly.py [--full]
-> out/funnel_in_ring_frame.step (the funnel, placed) and out/holder_seated_in_ring_frame.stl; --full also writes
out/funnel_on_ring_assembly.step (ring + funnel + holder, ~80 MB: the meshes become many faces)
"""
import math
import os
import sys

import cadquery as cq
from OCP.StlAPI import StlAPI_Reader
from OCP.TopoDS import TopoDS_Shape

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import funnel  # noqa: E402

RING = os.path.join(HERE, '..', 'ring', 'Ring_Section.stl')
HOLDER = os.path.join(HERE, '..', 'ring', 'Swung_electromagnet_holder.stl')
PLATE_AZ_DEG = 60.0     # the plate the funnel goes on, in the section's frame (the plates sit every 30 deg at r 225)
TOP_Z = 3.0             # the ring's top surface (the top skin's upper face) in the section's frame, mm
STEEL_Z = 0.0           # the steel plate's face (under the top skin)


def stl_mm(path):
    shape = TopoDS_Shape()
    if not StlAPI_Reader().Read(shape, path):
        raise RuntimeError(f'could not read {path}')
    return cq.Shape.cast(shape).scale(1000.0)       # the STLs are in metres


def main():
    out = os.path.join(HERE, 'out')
    os.makedirs(out, exist_ok=True)
    p = funnel.P
    a = math.radians(PLATE_AZ_DEG)
    cx, cy = p['plate_r_ring'] * math.cos(a), p['plate_r_ring'] * math.sin(a)
    part = funnel.build(p)
    assy = cq.Assembly(name='funnel_on_ring')
    assy.add(stl_mm(RING), name='ring_section', color=cq.Color(0.6, 0.6, 0.6))
    # the funnel's frame: x radially out, y anticlockwise, z up from the ring's top surface
    assy.add(part, name='funnel', color=cq.Color(0.2, 0.45, 0.85, 0.6),
             loc=cq.Location(cq.Vector(cx, cy, TOP_Z), cq.Vector(0, 0, 1), PLATE_AZ_DEG))
    disc_bottom = STEEL_Z + p['magnet_below_disc']
    assy.add(stl_mm(HOLDER), name='magnet_holder_seated', color=cq.Color(0.85, 0.25, 0.2),
             loc=cq.Location(cq.Vector(cx, cy, disc_bottom), cq.Vector(0, 0, 1), PLATE_AZ_DEG))
    # the electromagnet itself (20 mm x 20 mm), under the disc, as a plain cylinder
    assy.add(cq.Workplane('XY').circle(10.0).extrude(p['magnet_below_disc']), name='electromagnet_20x20',
             color=cq.Color(0.3, 0.3, 0.3), loc=cq.Location(cq.Vector(cx, cy, STEEL_Z)))
    path = os.path.join(out, 'funnel_on_ring_assembly.step')
    assy.save(path)
    print('wrote', path, f'({os.path.getsize(path) / 1e6:.1f} MB)')
    print(f'funnel on the plate at {PLATE_AZ_DEG:.0f} deg: centre ({cx:.1f}, {cy:.1f}) mm, the zip-tie hole is the inner one '
          f'at {PLATE_AZ_DEG - 10:.0f} deg (clockwise side)')


def placed_parts(variant='v1'):
    """The funnel (a solid STEP) and the seated holder (an STL) in the ring section's own frame, mm: imported into the
    Onshape Part Studio the section came from (if its STL kept that origin), they land on the plate at PLATE_AZ_DEG."""
    out = os.path.join(HERE, 'out')
    p = funnel.VARIANTS[variant]
    a = math.radians(PLATE_AZ_DEG)
    cx, cy = p['plate_r_ring'] * math.cos(a), p['plate_r_ring'] * math.sin(a)
    f = funnel.build(p).val().rotate((0, 0, 0), (0, 0, 1), PLATE_AZ_DEG).translate(cq.Vector(cx, cy, TOP_Z))
    stem = 'funnel_in_ring_frame' if variant == 'v1' else f'funnel_{variant}_in_ring_frame'
    cq.exporters.export(cq.Workplane().add(f), os.path.join(out, stem + '.step'))
    h = stl_mm(HOLDER).rotate((0, 0, 0), (0, 0, 1), PLATE_AZ_DEG).translate(
        cq.Vector(cx, cy, STEEL_Z + p['magnet_below_disc']))
    mag = cq.Workplane('XY').circle(10.0).extrude(p['magnet_below_disc']).val().translate(cq.Vector(cx, cy, STEEL_Z))
    cq.exporters.export(cq.Workplane().add(h).add(mag), os.path.join(out, 'holder_seated_in_ring_frame.stl'))
    for n in (stem + '.step', 'holder_seated_in_ring_frame.stl'):
        print('wrote', os.path.join(out, n), f'({os.path.getsize(os.path.join(out, n)) / 1e6:.2f} MB)')


if __name__ == '__main__':
    for v in funnel.VARIANTS:
        placed_parts(v)
    if '--full' in sys.argv:
        main()
