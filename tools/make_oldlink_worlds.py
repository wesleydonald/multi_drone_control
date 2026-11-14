#!/usr/bin/env python3
"""The twin's worlds with the quadrotor masses of before 4 Nov 2026 (Wesley 6 Nov: the claims with returns fly them).

tools/make_carry_worlds.py generates every rig world and drone model with the masses matched to the hardware on 4 Nov
(cable 0.002 + magnet 0.038 kg on a 0.510 kg airframe). With those, a returning quad needs ~35 s to settle its cable end on
the plate against ~17 s before (registry R3907 / R3908, feed F058). This writes `<world>_oldlinks.sdf` and
`models/x3_rig_drone{i}_oldlinks.sdf` beside them with the earlier split (0.040 along the cable + 0.035 magnet on a
0.475 airframe; each drone 0.55 kg either way), from the same generator, for the worlds the return claims fly.

    python3 tools/make_oldlink_worlds.py [--check]
"""
import argparse
import os
import re
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_carry_worlds as m  # noqa: E402
import make_carry_worlds_p115 as p115  # noqa: E402  (its worlds read m.RIG_ARGS at call time)

OLD = dict(ROD_MASS=0.040, MAGNET_MASS=0.035, AIRFRAME_MASS=0.475)
WORLDS = ['eight_rigid_ground_rig_R50_uh_L100.sdf', 'eight_rigid_ground_rig_R50_m115_uh_L100.sdf',
          'eight_rigid_ground_rig_R50_L100.sdf', 'nine_rigid_ground_rig_R53_L100.sdf', 'four_rigid_ground_rig_L100.sdf',
          'three_rigid_ground_rig_T_L100.sdf', 'three_rigid_ground_rig_T_push_L100.sdf',
          # X1 (Wesley 6 Nov ~11:35: its square-to-T cell depends on the masses)
          'four_rigid_ground_rig_3915_L100.sdf', 'four_rigid_ground_rig_3915_L100_p115.sdf']
SUFFIX = '_oldlinks'
MODEL = re.compile(r'models/(x3_rig_drone\d+)\.sdf')


def generate_old(out_dir):
    for k, v in OLD.items():
        setattr(m, k, v)
    m.RIG_BASE_MASS = m.AIRFRAME_MASS - 4 * m.ROTOR_MASS
    args = list(m.RIG_ARGS)                       # built at import with the new masses
    args[args.index('--rod-mass') + 1] = str(m.ROD_MASS)
    args[args.index('--magnet-mass') + 1] = str(m.MAGNET_MASS)
    m.RIG_ARGS = args
    m.generate(out_dir)
    p115.generate(out_dir)


def outputs(tmp):
    """{path under simulation_assets: text} of the old-mass worlds (their drone models renamed) and those models."""
    out = {}
    for w in WORLDS:
        text = open(os.path.join(tmp, w)).read()
        for name in set(MODEL.findall(text)):
            out[os.path.join('models', name + SUFFIX + '.sdf')] = open(os.path.join(tmp, 'models', name + '.sdf')).read()
        out[w[:-4] + SUFFIX + '.sdf'] = MODEL.sub(lambda g: f'models/{g.group(1)}{SUFFIX}.sdf', text)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true', help='fail if a written file is missing or stale')
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        generate_old(tmp)
        files = outputs(tmp)
    stale = []
    for rel, text in sorted(files.items()):
        path = os.path.join(m.ASSETS, rel)
        if a.check:
            if not os.path.exists(path) or open(path).read() != text:
                stale.append(rel)
            continue
        with open(path, 'w') as f:
            f.write(text)
        print('wrote', rel)
    if a.check:
        print('stale:' if stale else 'ok', ' '.join(stale))
        sys.exit(1 if stale else 0)


if __name__ == '__main__':
    main()
