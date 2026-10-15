#!/usr/bin/env python3
"""tools/make_attach_worlds_L100.py -- the combined M1 attach world (free start, Tejen's ball) with 1 m cables:
the carriers' rods and QUAD4's magnet arm 1.0 m (the supervisor's hardware for the week of 19 Oct; plan
2026-10-15 A1). make_attach_worlds.py with its cable length, the carriers' generator options and the ball's spot
swapped (the ball moves out: 1 m rods lay the carriers 1.2 m from the centre); new file names, so every existing
world and model is untouched.

    python3 tools/make_attach_worlds_L100.py [--check]

Writes simulation_assets/three_rigid_ground_rig_attach_pickup_L100.sdf and
models/x3_rig_drone3_magnet_pickup_L100.sdf.
"""
import argparse
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_attach_worlds as maw  # noqa: E402

CABLE_LEN = 1.0
SUFFIX = '_L100'
WORLD = maw.PICKUP_WORLD.replace('.sdf', f'{SUFFIX}.sdf')


def generate(out_dir):
    """Write the variant world and its newcomer model under out_dir (ASSETS layout); return their paths."""
    maw.CABLE_LEN = CABLE_LEN
    maw.RIG_ARGS = list(maw.RIG_ARGS) + ['--cable-len', str(CABLE_LEN)]     # later options win
    maw.BALL_XY = (1.6, 2.0)
    maw.MODELS = {k: v.replace('.sdf', f'{SUFFIX}.sdf') for k, v in maw.MODELS.items()}
    maw.PICKUP_MODELS = {k: v.replace('.sdf', f'{SUFFIX}.sdf') for k, v in maw.PICKUP_MODELS.items()}
    model = os.path.join('models', maw.PICKUP_MODELS['free'])
    with tempfile.TemporaryDirectory() as tmp:
        maw.generate(tmp)
        out = {model: open(os.path.join(tmp, model)).read(),
               WORLD: open(os.path.join(tmp, maw.PICKUP_WORLD)).read()}
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
        os.makedirs(os.path.join(tmp, 'models'))
        stale = [rel for rel in generate(tmp)
                 if not os.path.exists(os.path.join(maw.ASSETS, rel))
                 or open(os.path.join(tmp, rel)).read() != open(os.path.join(maw.ASSETS, rel)).read()]
    for rel in stale:
        print(f'stale: simulation_assets/{rel}')
    return 1 if stale else 0


if __name__ == '__main__':
    sys.exit(main())
