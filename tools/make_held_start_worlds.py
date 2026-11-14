#!/usr/bin/env python3
"""tools/make_held_start_worlds.py -- the held-object twin world (QUAD4 holding the ball from spawn, 1 m rods) with
QUAD4 starting further out, at 9, 1 or 5 o'clock (Wesley 12 Nov: the 13 Nov trials start QUAD4 at 9, 1 and 5 o'clock,
the carriers on 3/7/11, the re-attach always on plate 9).

    python3 tools/make_held_start_worlds.py [--check]

Reads simulation_assets/three_rigid_ground_rig_attach_held_pickup_L100.sdf (QUAD4 at (0, 1.4) on plate 9's line, the
ball on its tip 0.999 m further along +y) and writes ..._held_pickup_s9_L100.sdf, ..._s1_ and ..._s5_: QUAD4 at RADIUS
(1.9 m) from the ring centre on plate 9's, 1's or 5's line (the rig's held starts on 12 Nov were 2.1-2.2 m out; 1.9 keeps
1 and 5 o'clock inside the 1.8 m position limit) (clockwise from +x seen from above, 30 deg per plate), facing
+x as at 9 o'clock (his reference holds yaw 0), the ball on its tip 0.999 m along +y as in the source world. Nothing
else changes.
"""
import argparse
import math
import os
import re
import sys

ASSETS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'simulation_assets')
SRC = 'three_rigid_ground_rig_attach_held_pickup_L100.sdf'
RADIUS = 1.9
QUAD_POSE = re.compile(r'(<name>x3_drone3</name>\s*<pose>)(\S+) (\S+) (\S+)( 0 0 0</pose>)')
BALL_POSE = re.compile(r'(<uri>models/rig_pickup_ball\.sdf</uri>\s*<pose>)(\S+) (\S+) (\S+)( 0 0 0</pose>)')


def plate_xy(plate, r):
    a = math.radians(-30.0 * plate)
    return r * math.cos(a), r * math.sin(a)


def variant(text, plate):
    q, b = QUAD_POSE.search(text), BALL_POSE.search(text)
    if not q or not b:
        raise SystemExit('QUAD4 or the ball pose not found in ' + SRC)
    qx, qy = float(q.group(2)), float(q.group(3))
    r = RADIUS
    dx, dy = float(b.group(2)) - qx, float(b.group(3)) - qy          # the ball on the tip, relative to QUAD4
    x, y = plate_xy(plate, r)
    text = QUAD_POSE.sub(lambda m: f'{m.group(1)}{x:.6f} {y:.6f} {m.group(4)}{m.group(5)}', text, count=1)
    text = BALL_POSE.sub(lambda m: f'{m.group(1)}{x + dx:.6f} {y + dy:.6f} {m.group(4)}{m.group(5)}', text, count=1)
    note = (f'<!-- HELD START AT {plate} O\'CLOCK (tools/make_held_start_worlds.py, Wesley 12 Nov): QUAD4 at '
            f'({x:.3f}, {y:.3f}), {r:.2f} m out on plate {plate}\'s line, facing +x; the ball on its tip at '
            f'({x + dx:.3f}, {y + dy:.3f}). Otherwise {SRC}. -->\n')
    return text.replace('<world ', note + '  <world ', 1) if '<world ' in text else note + text


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true', help='compare with the files on disk, write nothing')
    a = ap.parse_args()
    src = open(os.path.join(ASSETS, SRC)).read()
    stale = []
    for plate in (9, 1, 5):
        name = SRC.replace('_held_pickup_L100', f'_held_pickup_s{plate}_L100')
        out = variant(src, plate)
        path = os.path.join(ASSETS, name)
        if a.check:
            if not os.path.exists(path) or open(path).read() != out:
                stale.append(name)
        else:
            with open(path, 'w') as f:
                f.write(out)
            print('wrote', path)
    if stale:
        print('stale:', ', '.join(stale))
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
