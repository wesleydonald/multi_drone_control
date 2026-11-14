#!/usr/bin/env python3
"""The square attach world on the 0.86 kg M2A ring with the _p115 worlds' pickup plate (Q011, Wesley 9 Nov 2026).

sq_rigid_ground_rig_attach_pickup_L100.sdf (the M2A ring) carries his ball (0.1 kg, r 0.08) as the pickup object;
sq_rigid_ground_rig_attach_pickup_L100_p115.sdf (the 1.15 kg ring) carries the 100 x 100 x 25 mm 0.15 kg plate. The
4.3 pickup-then-join cell re-flies on the M2A ring with the plate, so only the ring changes: this writes
sq_ / three_rigid_ground_rig_attach_pickup_L100_plate.sdf = the M2A world with its ball block replaced by the p115
world's plate block (where that world has it; the same model name, payload_model). Fly them with partner
geometry:=rig_twin_L100_plate.

    python3 tools/make_m2a_plate_world.py [--check]
"""
import argparse
import os
import re
import sys

ASSETS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'simulation_assets')
# (the M2A world, the p115 world whose plate block it takes, the world written): square and triangle (Wesley 9 Nov: every
# Q011 cell on a plate world, so only the ring changes)
LAYOUTS = {
    'sq': ('sq_rigid_ground_rig_attach_pickup_L100.sdf', 'sq_rigid_ground_rig_attach_pickup_L100_p115.sdf',
           'sq_rigid_ground_rig_attach_pickup_L100_plate.sdf'),
    'tri': ('three_rigid_ground_rig_attach_pickup_L100.sdf', 'three_rigid_ground_rig_attach_pickup_L100_p115.sdf',
            'three_rigid_ground_rig_attach_pickup_L100_plate.sdf'),
}


def _block(text, marker, uri):
    """The object's block: from its comment marker to the </include> that closes the include of uri."""
    i = text.index(marker)
    j = text.index(uri, i)
    k = text.index('</include>', j) + len('</include>')
    return i, k


def build(base_name, plate_from):
    base = open(os.path.join(ASSETS, base_name)).read()
    src = open(os.path.join(ASSETS, plate_from)).read()
    bi, bk = _block(base, "<!-- ===== TEJEN'S BALL", 'models/rig_pickup_ball.sdf')
    pi, pk = _block(src, '<!-- ===== THE PICKUP PLATE', 'models/rig_pickup_plate.sdf')
    out = base[:bi] + src[pi:pk] + base[bk:]
    assert 'rig_pickup_ball' not in out and out.count('<uri>models/rig_pickup_plate.sdf</uri>') == 1
    assert re.search(r'<model name="payload"', out)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='fail if the written world is missing or stale')
    a = ap.parse_args()
    stale = []
    for base_name, plate_from, out in LAYOUTS.values():
        text = build(base_name, plate_from)
        path = os.path.join(ASSETS, out)
        if a.check:
            if not (os.path.exists(path) and open(path).read() == text):
                stale.append(out)
            continue
        with open(path, 'w') as fh:
            fh.write(text)
        print('wrote', out)
    if a.check:
        print('stale: ' + ' '.join(stale) if stale else 'ok')
        sys.exit(1 if stale else 0)


if __name__ == '__main__':
    main()
