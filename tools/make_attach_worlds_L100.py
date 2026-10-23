#!/usr/bin/env python3
"""tools/make_attach_worlds_L100.py -- the twin attach family with 1 m cables: the carriers' rods and QUAD4's magnet arm
1.0 m (the supervisor's hardware for the week of 19 Oct; plan 2026-10-15 A1; every attach world since 22 Oct).
make_attach_worlds.py with its cable length, the carriers' generator options and the ball's spot swapped (the ball
moves out: 1 m rods lay the carriers 1.2 m from the centre); every world and newcomer model is written as
<name>_L100.sdf, so the 0.55 m family is untouched. The ball copy is shared (identical).

    python3 tools/make_attach_worlds_L100.py [--check]

Writes, under simulation_assets/: three_rigid_ground_rig_attach{,_welded,_pickup,_welded_pickup,_held_pickup,_push,
_pickup_push,_welded_pickup_push}_L100.sdf and models/x3_rig_drone3_magnet{,_welded,_pickup,_welded_pickup}_L100.sdf.
"""
import argparse
import contextlib
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_attach_worlds as maw  # noqa: E402

CABLE_LEN = 1.0
SUFFIX = '_L100'
WORLD = maw.PICKUP_WORLD.replace('.sdf', f'{SUFFIX}.sdf')


@contextlib.contextmanager
def one_metre():
    """make_attach_worlds' settings for the 1 m family, restored afterwards (its own tests import it too)."""
    keep = {k: getattr(maw, k) for k in ('CABLE_LEN', 'RIG_ARGS', 'BALL_XY', 'MODELS', 'PICKUP_MODELS')}
    maw.CABLE_LEN = CABLE_LEN
    maw.RIG_ARGS = list(keep['RIG_ARGS']) + ['--cable-len', str(CABLE_LEN)]     # later options win
    maw.BALL_XY = (1.6, 2.0)
    maw.MODELS = {k: v.replace('.sdf', f'{SUFFIX}.sdf') for k, v in keep['MODELS'].items()}
    maw.PICKUP_MODELS = {k: v.replace('.sdf', f'{SUFFIX}.sdf') for k, v in keep['PICKUP_MODELS'].items()}
    try:
        yield
    finally:
        for k, v in keep.items():
            setattr(maw, k, v)


def generate(out_dir):
    """Write the 1 m worlds and newcomer models under out_dir (ASSETS layout); return their paths."""
    ball = os.path.join('models', maw.BALL_COPY)
    with one_metre(), tempfile.TemporaryDirectory() as tmp:
        out = {(rel if rel.startswith('models') else rel.replace('.sdf', f'{SUFFIX}.sdf')):
               open(os.path.join(tmp, rel)).read() for rel in maw.generate(tmp) if rel != ball}
    os.makedirs(os.path.join(out_dir, 'models'), exist_ok=True)
    for rel, text in out.items():
        with open(os.path.join(out_dir, rel), 'w') as f:
            f.write(text)
    return list(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true', help='fail if a file on disk differs from a fresh generation')
    a = ap.parse_args()
    if not a.check:
        for rel in generate(maw.ASSETS):
            print(f'wrote simulation_assets/{rel}')
        return 0
    with tempfile.TemporaryDirectory() as tmp:
        stale = [rel for rel in generate(tmp)
                 if not os.path.exists(os.path.join(maw.ASSETS, rel))
                 or open(os.path.join(tmp, rel)).read() != open(os.path.join(maw.ASSETS, rel)).read()]
    for rel in stale:
        print(f'stale: simulation_assets/{rel}')
    if not stale:
        print('rig 1 m attach worlds and newcomer models up to date')
    return 1 if stale else 0


if __name__ == '__main__':
    sys.exit(main())
