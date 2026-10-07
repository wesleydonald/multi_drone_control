#!/usr/bin/env python3
"""
tools/make_carry_worlds.py
--------------------------
Generate the rig-twin carry worlds and their drone models, so the sim geometry is
the rig's as flown on 30 Sep 2026 (docs/decisions.md; rig-thrust-map-and-geometry):

  drone      0.475 kg airframe with pack over all links  -> models/x3_rig_drone{i}.sdf
  rod        0.55 m from a ball 4 cm below the drone centre to the magnet, ball at
             both ends; rod + magnet 0.075 kg = 0.040 along the rod + 0.035 magnet at
             the payload end (split not weighed; the total is)
  ring       0.86 kg, outer diameter 0.51 m, 40 mm thick, magnets at r 0.225 flush
             with the top face (attach_z 0.020 above the CoG)

As the planner sees it: cable_len 0.55, attach_radius 0.225, drone_mass 0.55,
pivot_offset_z -0.04.

The legacy worlds (0.5 m rods, r 0.25, centre pivot, x3_drone*.sdf; in old_worlds/) are untouched:
the generator's defaults still reproduce them byte for byte.

Usage:  python3 tools/make_carry_worlds.py [--check]

Both worlds and models are GENERATED -- edit this script, not them. `--check`
regenerates into a temp dir and fails if anything on disk is stale.
"""
import argparse
import os
import re
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(REPO, 'simulation_assets')
GEN = os.path.join(ASSETS, 'generate_rigid_world.py')

PIVOT_DZ = 0.04
CABLE_LEN = 0.55
ROD_MASS = 0.040
MAGNET_MASS = 0.035
ATTACH_RADIUS = 0.225
RING_OUTER_R = 0.255
RING_THICKNESS = 0.040
AIRFRAME_MASS = 0.475
LEGACY_BASE_MASS, ROTOR_MASS = 0.6, 0.01
RIG_BASE_MASS = AIRFRAME_MASS - 4 * ROTOR_MASS

# What check_geometry.world_geometry must read back from every rig world.
EXPECT = {'cable_len': CABLE_LEN, 'attach_radius': ATTACH_RADIUS,
          'attach_z': RING_THICKNESS / 2.0, 'pivot_dz': PIVOT_DZ,
          'rod_mass': ROD_MASS + MAGNET_MASS, 'drone_airframe_mass': AIRFRAME_MASS,
          'drone_mass': AIRFRAME_MASS + ROD_MASS + MAGNET_MASS, 'load_mass': 0.86}

WORLDS = {
    'four_rigid_ground_rig.sdf': ['--n', '4'],
    'four_rigid_ground_rig_3915.sdf': ['--n', '4', '--azimuths', '30,90,150,270'],
    'three_rigid_ground_rig.sdf': ['--n', '3'],
    # ring-mass mismatch neighbours: the planner keeps 0.86 (card A, rig_twin_hover_mismatch*)
    'four_rigid_ground_rig_m070.sdf': ['--n', '4', '--payload-mass', '0.70'],
    'four_rigid_ground_rig_m100.sdf': ['--n', '4', '--payload-mass', '1.0'],
}
RIG_ARGS = ['--cable-len', str(CABLE_LEN), '--detachable', '--ground-start',
            '--attach-radius', str(ATTACH_RADIUS), '--pivot-dz', str(PIVOT_DZ),
            '--rod-mass', str(ROD_MASS), '--magnet-mass', str(MAGNET_MASS),
            '--ring-outer-r', str(RING_OUTER_R), '--ring-thickness', str(RING_THICKNESS),
            '--drone-model', 'x3_rig']


def rig_model(legacy_text, i):
    """x3_drone{i}.sdf with the base_link mass cut to RIG_BASE_MASS and its inertia
    scaled by the same ratio; everything else (names, topics, motors) unchanged."""
    k = RIG_BASE_MASS / LEGACY_BASE_MASS
    base = legacy_text.index('<link name="base_link">')
    end = legacy_text.index('</inertial>', base)
    blk = legacy_text[base:end]
    if blk.count(f'<mass>{LEGACY_BASE_MASS}</mass>') != 1:
        raise SystemExit(f'x3_drone{i}.sdf: base_link mass is not {LEGACY_BASE_MASS}')
    blk = blk.replace(f'<mass>{LEGACY_BASE_MASS}</mass>', f'<mass>{RIG_BASE_MASS:.3f}</mass>')
    for tag in ('ixx', 'iyy', 'izz'):
        v = float(re.search(rf'<{tag}>([0-9.e-]+)</{tag}>', blk).group(1))
        blk = re.sub(rf'<{tag}>[0-9.e-]+</{tag}>', f'<{tag}>{v * k:.12f}</{tag}>', blk, count=1)
    hdr = '<sdf version="1.6">'
    note = (f'\n    <!-- Rig twin of x3_drone{i}.sdf (tools/make_carry_worlds.py): airframe with '
            f'pack {AIRFRAME_MASS} kg over all links\n         (base {RIG_BASE_MASS:.3f} + four '
            f'{ROTOR_MASS} rotors), base inertia scaled by {RIG_BASE_MASS:.3f}/{LEGACY_BASE_MASS}. '
            'Rod and magnet are on the rod. -->')
    text = legacy_text[:base] + blk + legacy_text[end:]
    if not text.startswith(hdr):
        raise SystemExit(f'x3_drone{i}.sdf: unexpected header')
    return text.replace(hdr, hdr + note, 1)


def generate(out_dir):
    """Write every rig world and model under out_dir (same layout as ASSETS)."""
    os.makedirs(os.path.join(out_dir, 'models'), exist_ok=True)
    written = []
    for i in range(4):
        with open(os.path.join(ASSETS, 'models', f'x3_drone{i}.sdf')) as f:
            text = rig_model(f.read(), i)
        rel = os.path.join('models', f'x3_rig_drone{i}.sdf')
        with open(os.path.join(out_dir, rel), 'w') as f:
            f.write(text)
        written.append(rel)
    for name, args in WORLDS.items():
        subprocess.run([sys.executable, GEN, *args, *RIG_ARGS,
                        '--out', os.path.join(out_dir, name)],
                       check=True, cwd=ASSETS, stdout=subprocess.DEVNULL)
        written.append(name)
    return written


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--check', action='store_true',
                    help='regenerate into a temp dir and fail if a file on disk differs')
    a = ap.parse_args()
    if not a.check:
        for rel in generate(ASSETS):
            print(f'wrote simulation_assets/{rel}')
        return 0
    stale = []
    with tempfile.TemporaryDirectory() as tmp:
        for rel in generate(tmp):
            with open(os.path.join(tmp, rel), 'rb') as f:
                want = f.read()
            path = os.path.join(ASSETS, rel)
            have = open(path, 'rb').read() if os.path.exists(path) else None
            if have != want:
                stale.append(rel)
    for rel in stale:
        print(f'stale: simulation_assets/{rel}')
    if stale:
        print('rerun tools/make_carry_worlds.py')
        return 1
    print('rig carry worlds and models up to date')
    return 0


if __name__ == '__main__':
    sys.exit(main())
