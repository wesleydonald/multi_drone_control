#!/usr/bin/env python3
"""tools/make_carry_worlds_p115.py -- the 1 m carry worlds for the payload of 28 Oct (registry RIG-1028-payload).

The new payload: 1.15 kg, a flat 12-sided plate ring 0.51 m across with a 0.27 m netted hole, magnets at r 0.204 m,
centre of mass ~1 cm off the floor (Motive pivot at rest 0.013 m), plate faces ~0.025 m above the pivot, inertia
estimated as a flat annulus (r 0.255 / 0.135): ixx 0.024, izz 0.049 kg m^2 (real.yaml).

make_carry_worlds.py's rig twin (RIG_ARGS: drones, rods, magnets, thrust-map models) with 1 m cables and that ring:
--payload-mass 1.15, --attach-radius 0.204, --attach-z 0.025, --ring-outer-r 0.255 and a 0.026 m thick ring, whose
mid-plane (the generator's CoG) then rests 0.013 m off the floor. The generator's ring is a 60 mm wide annulus
(ixx 0.0297, izz 0.0593 at 1.15 kg), so the payload inertia is rewritten to the estimate the planner flies: izz 0.049
and ixx = izz/2 + m h^2/12 = 0.0246, since 0.024 / 0.049 breaks ixx + iyy >= izz and Gazebo refuses the link.
Every world is <0.86 kg name>_p115.sdf beside the old ones, which stay untouched.

    python3 tools/make_carry_worlds_p115.py [--check]
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import make_carry_worlds as mcw  # noqa: E402

LOAD_MASS = 1.15
RING_THICKNESS = 0.026
LOAD_IZZ = 0.049
LOAD_IXX = LOAD_IZZ / 2.0 + LOAD_MASS * RING_THICKNESS ** 2 / 12.0
P115_ARGS = ['--cable-len', '1.0', '--tether-pose-hz', '60', '--payload-mass', str(LOAD_MASS),
             '--attach-radius', '0.204', '--attach-z', '0.025', '--ring-outer-r', '0.255', '--ring-thickness', str(RING_THICKNESS)]
# the azimuth sets of the 1 m baseline and chain worlds (generator azimuths count counter-clockwise)
WORLDS = {
    'four_rigid_ground_rig_L100_p115.sdf': ['--n', '4'],
    'three_rigid_ground_rig_L100_p115.sdf': ['--n', '3'],
    'two_rigid_ground_rig_L100_p115.sdf': ['--n', '2'],
    'three_rigid_ground_rig_T_L100_p115.sdf': ['--n', '3', '--azimuths', '0,270,180'],
    # the triangle 3/11/7 the rig flies its 3-quad baselines on (docs/rig/fri_2026-10-30.md; clockwise 90,330,210)
    'three_rigid_ground_rig_tri_L100_p115.sdf': ['--n', '3', '--azimuths', '270,30,150'],
    # the M1 layout (carriers 3/11/7 + plate 9; make_carry_worlds' _3915) for X1's 1.15 kg cell (card 2026-10-30_x1_release_ablation)
    'four_rigid_ground_rig_3915_L100_p115.sdf': ['--n', '4', '--azimuths', '30,90,150,270'],
}
INERTIA = re.compile(r'(<model name="payload">.*?<inertia>)<ixx>[^<]+</ixx>(<ixy>0</ixy><ixz>0</ixz>)'
                     r'<iyy>[^<]+</iyy>(<iyz>0</iyz>)<izz>[^<]+</izz>', re.S)


def generate(out_dir):
    written = []
    for name, args in WORLDS.items():
        path = os.path.join(out_dir, name)
        subprocess.run([sys.executable, mcw.GEN, *args, *mcw.RIG_ARGS, *P115_ARGS, '--out', path],
                       check=True, cwd=mcw.ASSETS, stdout=subprocess.DEVNULL)
        with open(path) as f:
            text = f.read()
        text, k = INERTIA.subn(rf'\g<1><ixx>{LOAD_IXX:.4e}</ixx>\g<2><iyy>{LOAD_IXX:.4e}</iyy>\g<3>'
                               rf'<izz>{LOAD_IZZ:.4e}</izz>', text, count=1)
        if k != 1:
            raise SystemExit(f'{name}: no payload inertia to rewrite')
        with open(path, 'w') as f:
            f.write(text)
        written.append(name)
    return written


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true', help='fail if a file on disk differs from a fresh generation')
    a = ap.parse_args()
    if not a.check:
        for rel in generate(mcw.ASSETS):
            print(f'wrote simulation_assets/{rel}')
        return 0
    with tempfile.TemporaryDirectory() as tmp:
        stale = [rel for rel in generate(tmp)
                 if not os.path.exists(os.path.join(mcw.ASSETS, rel))
                 or open(os.path.join(tmp, rel)).read() != open(os.path.join(mcw.ASSETS, rel)).read()]
    for rel in stale:
        print(f'stale: simulation_assets/{rel}')
    if not stale:
        print('1.15 kg payload carry worlds up to date')
    return 1 if stale else 0


if __name__ == '__main__':
    sys.exit(main())
