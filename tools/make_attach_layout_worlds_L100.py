#!/usr/bin/env python3
"""tools/make_attach_layout_worlds_L100.py -- the 1 m attach worlds with other carrier layouts (Wesley 27 Oct: the
square and the pair re-attach). make_attach_worlds_L100.py with the carriers moved; QUAD4 still joins rig plate 9.

    sq:    carriers on rig plates 0/3/6 (the T of the square), QUAD4 on 9 -> the square
    pair:  carriers on rig plates 0/6, QUAD4 on 9 -> the T 0/6/9
    pair2: carriers on rig plates 0/6, QUAD4 welded on 9 and QUAD3 (drone_2) welded on 3, each with its own ring weld
           (QUAD3's on /drone_2/payload/*): the square of the 4 -> 2 -> 4 (Wesley 13 Nov); the welded start only

Writes simulation_assets/{sq,pair}_rigid_ground_rig_attach{_pickup,_welded_pickup}_L100.sdf and
pair2_rigid_ground_rig_attach_welded_pickup_L100.sdf with models/x3_rig_drone2_magnet_welded_L100.sdf; the newcomer
and ball models are the M1 family's (checked identical).

    python3 tools/make_attach_layout_worlds_L100.py [--check]
"""
import argparse
import math
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_attach_worlds as maw  # noqa: E402
import make_attach_worlds_L100 as L100  # noqa: E402

# internal azimuths are counter-clockwise: rig plate p sits at -30 p deg
LAYOUTS = {'sq': ('0,270,180', '0/3/6'), 'pair': ('0,180', '0/6'), 'pair2': ('0,180', '0/6')}
KEEP = ('three_rigid_ground_rig_attach_pickup.sdf', 'three_rigid_ground_rig_attach_welded_pickup.sdf')
# further partners welded from spawn, (drone id, rig plate), and the worlds such a layout keeps (they start welded)
EXTRA_PARTNERS = {'pair2': ((2, 3),)}
LAYOUT_KEEP = {'pair2': ('three_rigid_ground_rig_attach_welded_pickup.sdf',)}


def generate(layout):
    az, plates = LAYOUTS[layout]
    n = len(az.split(','))
    extra = EXTRA_PARTNERS.get(layout, ())

    def carrier_world():
        with tempfile.TemporaryDirectory() as tmp:
            out = os.path.join(tmp, 'w.sdf')
            subprocess.run([sys.executable, maw.GEN, '--n', str(n), '--azimuths', az, *maw.RIG_ARGS, '--out', out],
                           check=True, cwd=maw.ASSETS, stdout=subprocess.DEVNULL)
            with open(out) as f:
                return f.read()

    def ball_clearance(base, start='free'):
        # make_attach_worlds.ball_clearance for any number of carriers; a further partner counts as a carrier
        carriers = re.findall(r'<name>x3_drone[0-2]</name>\s*<pose>(\S+) (\S+)', base)
        if len(carriers) != n:
            raise SystemExit('carrier poses not found in the base world')
        bodies = [(float(x), float(y)) for x, y in carriers] + [maw.spawn('welded', p)[0][:2] for _, p in extra]
        near_carrier = min(math.dist(maw.BALL_XY, xy) for xy in bodies)
        (x, y, _), roll = maw.spawn(start)
        tip_y = y + maw.CABLE_LEN * math.sin(roll)
        lo, hi = min(y, tip_y), max(y, tip_y)
        return near_carrier, math.dist(maw.BALL_XY, (x, min(max(maw.BALL_XY[1], lo), hi)))

    keep = (maw.CARRIER_AZIMUTHS, maw.carrier_world, maw.ball_clearance, maw.EXTRA_PARTNERS)
    maw.CARRIER_AZIMUTHS, maw.carrier_world, maw.ball_clearance = az, carrier_world, ball_clearance
    maw.EXTRA_PARTNERS = extra
    try:
        with L100.one_metre(), tempfile.TemporaryDirectory() as tmp:
            rels = maw.generate(tmp)
            texts = {rel: open(os.path.join(tmp, rel)).read() for rel in rels}
            own_models = {os.path.join('models', maw.partner_model_name(d)) for d, _ in extra}
    finally:
        maw.CARRIER_AZIMUTHS, maw.carrier_world, maw.ball_clearance, maw.EXTRA_PARTNERS = keep
    out = {}
    for rel, text in texts.items():
        if rel in own_models:
            out[rel] = text
        elif rel.startswith('models'):
            with open(os.path.join(maw.ASSETS, rel)) as f:
                if f.read() != text:
                    raise SystemExit(f'{rel} differs from the M1 family: regenerate that first')
        elif rel in LAYOUT_KEEP.get(layout, KEEP):
            name = rel.replace('three_', f'{layout}_').replace('.sdf', f'{L100.SUFFIX}.sdf')
            text = text.replace('(rig plates 3/11/7)', f'(rig plates {plates})')
            out[name] = text.replace('three carriers', f'{n} carriers') if extra else text
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true', help='fail if a file on disk differs from a fresh generation')
    a = ap.parse_args()
    stale = []
    for layout in LAYOUTS:
        for name, text in generate(layout).items():
            path = os.path.join(maw.ASSETS, name)
            if a.check:
                if not os.path.exists(path) or open(path).read() != text:
                    stale.append(name)
            else:
                with open(path, 'w') as f:
                    f.write(text)
                print(f'wrote simulation_assets/{name}')
    if stale:
        print('stale: ' + ', '.join(stale))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
