#!/usr/bin/env python3
"""tools/make_stand_pickup_worlds.py -- the full-mission twin world (QUAD4 welded on plate 9, his ball to pick up) with
the ball raised on a stand (Wesley 12 Nov: the 13 Nov hero flight picks the object up from Thomas' stand, about ring
height, instead of the floor).

    python3 tools/make_stand_pickup_worlds.py [--check]

Reads simulation_assets/three_rigid_ground_rig_attach_welded_pickup_L100.sdf (the world of rig_fm9_12nov_twin: the ball
on the floor at (1.6, 2.0), its centre 0.08 m up) and writes ..._welded_pickup_stand100_L100.sdf, _stand115_ and
_stand130_: a static stand at the ball's x, y (a 0.20 x 0.20 x 0.02 m top on a 0.04 m column on a 0.30 x 0.30 x 0.02 m
foot) with its top at 1.00, 1.15 or 1.30 m, and the ball resting on it (its centre 0.08 m above the top). The stand
collides as the floor does (bits 0x03: the ball, QUAD4's magnet tip and body, the ring); the twin's rods have no
collision, so a cable through the stand shows only in the logs (QUAD4's body-to-tip line against the column). Nothing
else changes.
"""
import argparse
import os
import re
import sys

ASSETS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'simulation_assets')
SRC = 'three_rigid_ground_rig_attach_welded_pickup_L100.sdf'
TOPS = (1.00, 1.15, 1.30)          # m, the stand's top
BALL_R = 0.08                      # models/rig_pickup_ball.sdf
TOP_SIDE, TOP_T = 0.20, 0.02       # the stand's top plate
FOOT_SIDE, FOOT_T = 0.30, 0.02
COLUMN_R = 0.02
BITS = '0x03'                      # as the floor
BALL_POSE = re.compile(r'(<uri>models/rig_pickup_ball\.sdf</uri>\s*<pose>)(\S+) (\S+) (\S+)( 0 0 0</pose>)')


def _part(name, z, geometry):
    return (f'        <collision name="{name}"><pose>0 0 {z:.4f} 0 0 0</pose><geometry>{geometry}</geometry>'
            f'<surface><contact><collide_bitmask>{BITS}</collide_bitmask></contact></surface></collision>\n'
            f'        <visual name="{name}_visual"><pose>0 0 {z:.4f} 0 0 0</pose><geometry>{geometry}</geometry>'
            f'<material><ambient>0.35 0.35 0.4 1</ambient><diffuse>0.45 0.45 0.5 1</diffuse></material></visual>\n')


def stand_model(x, y, top):
    column = top - TOP_T - FOOT_T
    return (f'    <!-- ===== THE STAND (tools/make_stand_pickup_worlds.py): static, top {top:.2f} m at ({x:.3f}, {y:.3f}), '
            f'collides as the floor ({BITS}) ===== -->\n'
            f'    <model name="pickup_stand">\n'
            f'      <static>true</static>\n'
            f'      <pose>{x:.6f} {y:.6f} 0 0 0 0</pose>\n'
            f'      <link name="link">\n'
            + _part('foot', FOOT_T / 2, f'<box><size>{FOOT_SIDE} {FOOT_SIDE} {FOOT_T}</size></box>')
            + _part('column', FOOT_T + column / 2,
                    f'<cylinder><radius>{COLUMN_R}</radius><length>{column:.4f}</length></cylinder>')
            + _part('top', top - TOP_T / 2, f'<box><size>{TOP_SIDE} {TOP_SIDE} {TOP_T}</size></box>')
            + '      </link>\n'
            '    </model>\n')


def variant(text, top, xy=None):
    b = BALL_POSE.search(text)
    if not b or '  </world>' not in text:
        raise SystemExit('the ball pose or </world> not found in ' + SRC)
    x, y = (float(b.group(2)), float(b.group(3))) if xy is None else (float(xy[0]), float(xy[1]))
    text = BALL_POSE.sub(lambda m: f'{m.group(1)}{x:g} {y:g} {top + BALL_R:.6f}{m.group(5)}' if xy is not None
                         else f'{m.group(1)}{m.group(2)} {m.group(3)} {top + BALL_R:.6f}{m.group(5)}', text, count=1)
    text = text.replace('  </world>', stand_model(x, y, top) + '  </world>', 1)
    note = (f'<!-- THE BALL ON A STAND (tools/make_stand_pickup_worlds.py, Wesley 12 Nov): a static stand at '
            f'({x:.3f}, {y:.3f}) with its top at {top:.2f} m, the ball resting on it (centre {top + BALL_R:.3f} m). '
            f'Otherwise {SRC}. -->\n')
    return text.replace('<world ', note + '  <world ', 1)


def name_for(top, xy=None):
    at = '' if xy is None else f'_x{round(xy[0] * 100):03d}y{round(xy[1] * 100):03d}'
    return SRC.replace('_welded_pickup_L100', f'_welded_pickup_stand{round(top * 100):03d}{at}_L100')


# Wesley 13 Nov: the stand at (1.4, 1.4), top 1.15 m (the twin's ring centre at the origin)
PLACED = ((1.15, (1.4, 1.4)),)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true', help='compare with the files on disk, write nothing')
    a = ap.parse_args()
    src = open(os.path.join(ASSETS, SRC)).read()
    stale = []
    for top, xy in [(t, None) for t in TOPS] + list(PLACED):
        name = name_for(top, xy)
        out = variant(src, top, xy)
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
