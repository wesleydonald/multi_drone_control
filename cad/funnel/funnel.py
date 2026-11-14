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
    bore_r=36.0, bore_top=26.0, bore_wall=2.4,
    # the cone: the mouth's half-axes = the miss it catches + the disc's radius + a margin
    catch_radial=70.0, catch_tangential=45.0, margin=5.0,
    cone_angle_radial_deg=40.0,  # the shallowest wall (along the radius), from horizontal; the sideways wall is steeper
    cone_t=1.6,               # the cone's wall measured across it (4 perimeters of a 0.4 mm nozzle); a sideways offset of
                              # 0.8 mm, as first drawn, left 0.4-0.6 mm on a 32-45 deg wall: one printed line
    knee_fillet=15.0,         # outside round where the cone meets the bore wall (the corner that bends when the rim is hit)
    lead_in=2.0,              # inside round on the bore's top edge, so the disc slides in and out without catching
    # buttresses under the cone, out to rib_reach of the way to the rim: radial in and out (where the misses hit) and
    # sideways at +-96 deg, clear of the three bolt bosses (+-64, 129 deg) and the zip-tie hole (-129 deg)
    rib_t=2.0, rib_azimuths_deg=(0.0, 96.0, 180.0, -96.0), rib_reach=0.8,
    bolts=('in_ccw', 'out_cw', 'out_ccw'),   # the holes used (in_cw holds the net's zip tie)
    ziptie_notch_r=6.0, ziptie_notch_h=8.0,  # a notch round the zip-tie hole, so a wide base never sits on the tie
    # the flange and the nut bosses (M4: nut 7.0 mm across flats, 3.2 mm thick)
    ring_w=6.0, arm_w=10.0, flange_t=3.0,   # a ring round the bore and three arms out to the bolts (no flange near the zip tie)
    boss_r=6.5, boss_top=9.0, nut_af=7.0, nut_t=3.2, fit=0.3,
    density=1.27,             # PETG g/cm3 (PLA 1.24)
)

# Wesley 5 Nov: v1 slightly too big; v2 smaller, both kept so he can choose which to print. Wesley 6 Nov: v2's wall at
# 45 deg (was 32), the mouth as wide as before, so it stands taller
VARIANTS = {
    'v1': P,
    'v2': {**P, 'catch_radial': 50.0, 'catch_tangential': 35.0, 'cone_angle_radial_deg': 45.0, 'bore_top': 24.0},
    # Wesley 6 Nov: v1's mouth at 45 deg, reinforced less than v2 to keep its weight down: 3 printed lines across the
    # wall, ribs only radially in and out (where the misses fall), a thinner bore wall
    'v3': {**P, 'cone_angle_radial_deg': 45.0, 'cone_t': 1.2, 'bore_wall': 2.0,
           'rib_azimuths_deg': (0.0, 180.0), 'rib_reach': 0.6, 'rib_t': 1.6},
}
# Wesley 6 Nov: the magnet's wire fitting does not pass v3's 72 mm base -> v4: 85 mm across, 85 mm tall, printed in TPU
# (45 deg kept: TPU sags on overhangs and its grip needs the steeper wall); the mouth trimmed 3.5 mm a side to keep the
# height; the inner bolt hole (44.6 mm out) breaks into the base, so two outer bolts (full nut seats); the zip-tie notch
VARIANTS['v4'] = {**VARIANTS['v3'], 'bore_r': 42.5, 'bore_top': 24.0, 'catch_radial': 66.5, 'catch_tangential': 41.5,
                  'bolts': ('out_cw', 'out_ccw'), 'boss_r': 8.0, 'density': 1.21}
# Wesley 10 Nov: the base only as high as the electromagnet -> v5: v4 with the bore 20 mm tall (was 24). The holder's disc
# sits 17-20 mm up with the magnet on the steel, so a 20 mm bore still centres it (play 10.5 mm, as v4); 15 mm would leave
# the disc in the cone (play 12.5-15.5 mm, near the plate's ~14-18 mm full-contact radius). The mouth and wall as v4, so it
# stands 4 mm lower (81 mm)
VARIANTS['v5'] = {**VARIANTS['v4'], 'bore_top': 20.0}
# Wesley 10 Nov: v5 with the cone cut to about half -> v6: the cone half as tall at the same walls (45 deg radial, 59.5
# sideways): (a - bore_r) and (b - bore_r) halved, so catch_radial 36 (was 66.5), catch_tangential 23.5 (was 41.5); the
# base, bore, bolts and notch as v5
VARIANTS['v6'] = {**VARIANTS['v5'], 'catch_radial': 36.0, 'catch_tangential': 23.5}


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


def cone_offsets(p):
    """Sideways offsets of the cone's outer surface that give it cone_t across the wall, along the radius and sideways."""
    a, b, h = mouth(p)
    return (p['cone_t'] / math.sin(math.atan2(h, a - p['bore_r'])),
            p['cone_t'] / math.sin(math.atan2(h, b - p['bore_r'])))


def _knee(wp, z):
    """The circular edge where the bore's cylinder meets the cone (its centre is on the axis at height z)."""
    return wp.edges(cq.selectors.NearestToPointSelector((0.0, 0.0, z)))


def build(p):
    a, b, h = mouth(p)
    br, bw, bt, ft = p['bore_r'], p['bore_wall'], p['bore_top'], p['flange_t']
    da, db = cone_offsets(p)
    top = bt + h
    # the outer skin as one solid (the bore's cylinder, then the cone), its knee rounded on the outside
    outer = (cq.Workplane('XY').circle(br + bw).extrude(bt)
             .faces('>Z').workplane().circle(br + bw).workplane(offset=h).ellipse(a + da, b + db).loft(combine=True))
    outer = _knee(outer, bt).fillet(p['knee_fillet'])
    # the hollow: the bore, the cone's inside and the mouth, the bore's top edge rounded (a concave edge of the hollow)
    inner = (cq.Workplane('XY').workplane(offset=-1).circle(br).extrude(bt + 1)
             .faces('>Z').workplane().circle(br).workplane(offset=h).ellipse(a, b).loft(combine=True)
             .faces('>Z').workplane().ellipse(a, b).extrude(2.0))
    inner = _knee(inner, bt).fillet(p['lead_in'])
    part = outer.cut(inner)
    # a ring round the bore on the ring's top surface, and three arms out to the bolts (the zip-tie hole stays clear)
    used = [hh for hh in holes(p) if hh[0] in p['bolts']]
    part = part.union(cq.Workplane('XY').circle(br + p['ring_w']).circle(br).extrude(ft))
    for _, x, y in used:
        d, phi = math.hypot(x, y), math.degrees(math.atan2(y, x))
        arm = (cq.Workplane('XY').center((br + d) / 2.0, 0).rect(d - br, p['arm_w']).extrude(ft)
               .rotate((0, 0, 0), (0, 0, 1), phi))
        part = part.union(arm).union(cq.Workplane('XY').center(x, y).circle(p['boss_r']).extrude(p['boss_top']))
    # buttresses: the space under the cone along each azimuth, out to rib_reach of the way to the rim
    under = cq.Workplane('XY').ellipse(a + da, b + db).extrude(top).cut(outer)
    for az in p['rib_azimuths_deg']:
        t = math.radians(az)
        r_rim = a * b / math.hypot(b * math.cos(t), a * math.sin(t))
        reach = br + p['rib_reach'] * (r_rim - br)
        slab = (cq.Workplane('XY').center(reach / 2.0, 0).rect(reach, p['rib_t']).extrude(top)
                .rotate((0, 0, 0), (0, 0, 1), az))
        part = part.union(slab.intersect(under))
    part = part.cut(cq.Workplane('XY').workplane(offset=-1).circle(br).extrude(bt + 1.01))
    # the net's zip tie at in_cw: a notch through the base's foot (a wide base would otherwise sit on the tie)
    for name, x, y in holes(p):
        if name == 'in_cw':
            part = part.cut(cq.Workplane('XY').workplane(offset=-1).center(x, y).circle(p['ziptie_notch_r'])
                            .extrude(p['ziptie_notch_h'] + 1))
    # bolt holes, and hex pockets that hold the M4 nuts (bolts go in from under the ring)
    for _, x, y in used:
        part = part.cut(cq.Workplane('XY').workplane(offset=-1).center(x, y).circle(p['hole_d'] / 2 + 0.1)
                        .extrude(p['boss_top'] + 2))
        part = part.cut(cq.Workplane('XY').workplane(offset=p['boss_top'] - p['nut_t'] - p['fit'])
                        .center(x, y).polygon(6, (p['nut_af'] + p['fit']) / math.cos(math.radians(30)))
                        .extrude(p['nut_t'] + p['fit'] + 1))
    return part


def nut_clearance(p):
    """Height between each boss top and the cone's underside above it (room to drop the nut in), mm."""
    a, b, h = mouth(p)
    da, db = cone_offsets(p)
    out = []
    for name, x, y in holes(p):
        if name not in p['bolts']:
            continue
        t = math.atan2(y, x)
        r_rim = (a + da) * (b + db) / math.hypot((b + db) * math.cos(t), (a + da) * math.sin(t))
        r = math.hypot(x, y) - p['boss_r']                   # the boss's inner edge, nearest the cone's foot
        z = p['bore_top'] + max(0.0, r - (p['bore_r'] + p['bore_wall'])) / (r_rim - p['bore_r'] - p['bore_wall']) * h
        out.append(z - p['boss_top'])
    return out


def checks(p, part):
    a, b, h = mouth(p)
    vol = part.val().Volume() / 1000.0
    disc_bottom = -p['steel_below_top'] + p['magnet_below_disc']
    return {
        'mouth_radial_mm (full)': 2 * a, 'mouth_tangential_mm (full)': 2 * b,
        'height_top_mm (above the ring top)': p['bore_top'] + h,
        'wall_angle_radial_deg': p['cone_angle_radial_deg'],
        'wall_angle_tangential_deg': math.degrees(math.atan2(h, b - p['bore_r'])),
        'cone_wall_across_mm': p['cone_t'],
        'nut_clearance_under_cone_mm (lowest boss)': min(nut_clearance(p)),
        'valid_solid': part.val().isValid(),
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
