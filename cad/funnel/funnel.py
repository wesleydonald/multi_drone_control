"""The magnet funnel for one ring plate (Wesley, 5 Nov 2026), bolted to the ring through three of the four M4 holes around
the plate. It guides QUAD4's magnet holder (a 64 mm disc with the 20 x 20 mm electromagnet 20 mm below it) onto the
plate: the disc is centred by a short bore just above where it sits when the magnet is on the steel, and an oval cone
above the bore catches it. Sized from the triangle's 16 re-attach descents of 5 Nov (misses mostly radial, -8..+10 cm,
sideways mostly within 4 cm; docs/rig/notes_for_tejen_2026-11-05.md).

Frame (mm): the plate centre at the origin, x = radially outward, y = along the rim anticlockwise seen from above,
z = 0 on the ring's top surface (the top skin; the steel plate sits 3 mm below it, under the skin's 57 mm hole).
The holes (cad/ring/Ring_Section.stl): 4.5 mm at ring radius 200 and 250 mm, 10 deg either side of the plate. Wesley:
seen from outside the ring, "top" = inner, "left" = clockwise (toward the next plate number) = -y here; the top-left
hole holds the net's zip tie, so it is left clear (no arm there), and the funnel uses the other three. Bolts go in from
below (heads under the ring) into M4 nuts captured in pockets on the flange's bosses: the cone overhangs the holes.

Run (CadQuery lives in its own folder, apart from the ROS Python):
    PYTHONPATH=~/venvs/cad_pkgs python3 -s cad/funnel/funnel.py [--out cad/funnel/out]
"""
import argparse
import math
import os

import cadquery as cq

P = dict(
    # the ring (from Ring_Section.stl)
    plate_r_ring=225.0, hole_r_in=200.0, hole_r_out=250.0, hole_dtheta_deg=10.0, hole_d=4.5,
    steel_below_top=3.0,      # the steel plate's face under the ring's top surface
    # QUAD4's magnet holder (Swung_electromagnet_holder.stl + Wesley): the disc and the magnet under it
    disc_d=64.0, disc_t=3.0, magnet_below_disc=20.0,
    # the bore that centres the disc (play = bore_r - disc_d/2: the magnet lands within this of the plate centre)
    bore_r=36.0, bore_top=26.0, bore_wall=1.2,
    # the cone: the mouth's half-axes = the miss it catches + the disc's radius + a margin
    catch_radial=70.0, catch_tangential=45.0, margin=5.0,
    cone_angle_radial_deg=40.0,  # the shallowest wall (along the radius), from horizontal; the sideways wall is steeper
    cone_wall=0.8,            # two perimeters of a 0.4 mm nozzle
    # the flange and the nut bosses (M4: nut 7.0 mm across flats, 3.2 mm thick)
    ring_w=4.0, arm_w=10.0, flange_t=3.0,   # a ring round the bore and three arms out to the bolts (no flange near the zip tie)
    boss_r=6.5, boss_top=9.0, nut_af=7.0, nut_t=3.2, fit=0.3,
    density=1.27,             # PETG g/cm3 (PLA 1.24)
)

# Wesley 5 Nov: v1 slightly too big; v2 smaller and shallower, both kept so he can choose which to print
VARIANTS = {
    'v1': P,
    'v2': {**P, 'catch_radial': 50.0, 'catch_tangential': 35.0, 'cone_angle_radial_deg': 32.0, 'bore_top': 24.0},
}


def holes(p):
    """The four holes in the plate frame: (name, x, y)."""
    out = []
    for name, r in (('in', p['hole_r_in']), ('out', p['hole_r_out'])):
        for side, s in (('cw', -1.0), ('ccw', 1.0)):
            a = math.radians(p['hole_dtheta_deg'])
            out.append((f'{name}_{side}', r * math.cos(a) - p['plate_r_ring'], s * r * math.sin(a)))
    return out


def mouth(p):
    a = p['catch_radial'] + p['disc_d'] / 2.0 + p['margin']
    b = p['catch_tangential'] + p['disc_d'] / 2.0 + p['margin']
    h = (a - p['bore_r']) * math.tan(math.radians(p['cone_angle_radial_deg']))
    return a, b, h


def build(p):
    a, b, h = mouth(p)
    br, bw, bt, cw, ft = p['bore_r'], p['bore_wall'], p['bore_top'], p['cone_wall'], p['flange_t']
    used = [hh for hh in holes(p) if hh[0] != 'in_cw']
    # a ring round the bore on the ring's top surface, and three arms out to the bolts (the zip-tie hole stays clear)
    part = cq.Workplane('XY').circle(br + p['ring_w']).circle(br).extrude(ft)
    for _, x, y in used:
        d, phi = math.hypot(x, y), math.degrees(math.atan2(y, x))
        arm = (cq.Workplane('XY').center((br + d) / 2.0, 0).rect(d - br, p['arm_w']).extrude(ft)
               .rotate((0, 0, 0), (0, 0, 1), phi))
        part = part.union(arm).union(cq.Workplane('XY').center(x, y).circle(p['boss_r']).extrude(p['boss_top']))
    # the bore wall, then the cone (outer loft minus inner loft, open at both ends)
    part = part.union(cq.Workplane('XY').circle(br + bw).circle(br).extrude(bt))
    outer = (cq.Workplane('XY').workplane(offset=bt).circle(br + cw)
             .workplane(offset=h).ellipse(a + cw, b + cw).loft(combine=True))
    inner = (cq.Workplane('XY').workplane(offset=bt - 0.01).circle(br)
             .workplane(offset=h + 0.02).ellipse(a, b).loft(combine=True))
    part = part.union(outer.cut(inner))
    part = part.cut(cq.Workplane('XY').workplane(offset=-1).circle(br).extrude(bt + 1.01))
    # bolt holes, and hex pockets that hold the M4 nuts (bolts go in from under the ring)
    for _, x, y in used:
        part = part.cut(cq.Workplane('XY').workplane(offset=-1).center(x, y).circle(p['hole_d'] / 2 + 0.1)
                        .extrude(p['boss_top'] + 2))
        part = part.cut(cq.Workplane('XY').workplane(offset=p['boss_top'] - p['nut_t'] - p['fit'])
                        .center(x, y).polygon(6, (p['nut_af'] + p['fit']) / math.cos(math.radians(30)))
                        .extrude(p['nut_t'] + p['fit'] + 1))
    return part


def checks(p, part):
    a, b, h = mouth(p)
    vol = part.val().Volume() / 1000.0
    disc_bottom = -p['steel_below_top'] + p['magnet_below_disc']
    return {
        'mouth_radial_mm (full)': 2 * a, 'mouth_tangential_mm (full)': 2 * b,
        'height_top_mm (above the ring top)': p['bore_top'] + h,
        'wall_angle_radial_deg': p['cone_angle_radial_deg'],
        'wall_angle_tangential_deg': math.degrees(math.atan2(h, b - p['bore_r'])),
        'disc_seated_z_mm (bottom..top)': f"{disc_bottom:.1f}..{disc_bottom + p['disc_t']:.1f}",
        'disc_play_mm (magnet within this of the centre)': p['bore_r'] - p['disc_d'] / 2,
        'reaches_inward_to_ring_r_mm': p['plate_r_ring'] - a,
        'reaches_outward_to_ring_r_mm': p['plate_r_ring'] + a,
        'volume_cm3': vol, 'mass_g': vol * p['density'],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out'))
    ap.add_argument('--variant', default='v1', choices=sorted(VARIANTS))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    p = VARIANTS[a.variant]
    part = build(p)
    print(f'== funnel {a.variant}')
    for k, v in checks(p, part).items():
        print(f'{k:50s} {v if isinstance(v, str) else round(v, 1)}')
    for name, x, y in holes(p):
        print(f'hole {name:7s} x {x:6.1f}  y {y:6.1f}' + ('   (zip tie: left clear)' if name == 'in_cw' else ''))
    stem = 'funnel' if a.variant == 'v1' else f'funnel_{a.variant}'
    cq.exporters.export(part, os.path.join(a.out, stem + '.step'))
    cq.exporters.export(part, os.path.join(a.out, stem + '.stl'))
    print('wrote', os.path.join(a.out, stem + '.step/.stl'))


if __name__ == '__main__':
    main()
