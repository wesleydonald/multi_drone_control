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

Scalability S (card 2026-10-23_scalability) adds a bigger ring (R 0.5 m, outer 0.53, 0.9 kg) for 6 and 8 quads and
R 0.53 m for 9, with drone models 4-8 made from x3_drone0.sdf renamed.

Thesis R (card 2026-10-23_refly_1m, Wesley 23 Oct) adds the Tier B layouts and the push world at 1 m, and the rejoin
leaning-pair test the T at 1 m with the push tool; a world in PUSH_WORLDS also carries the runner's WRENCH tool
(ApplyLinkWrench), as the 0.55 m _push worlds do.

Thesis C on the paper's own geometry (card 2026-10-23_paper_method_compare, Wesley 23 Oct) adds the 1 m four with every
rod hinged on a 1 mm hub at the ring's centre (--hub-r), on the plates' plane: Quan et al.'s cables meet at the load's
centre. Not 0: the planner and the network take each quad's azimuth from its attach point. Quan et al. exactly (card
2026-10-23_quan_exact, Wesley 23 Oct) adds world C: that hub at the ring's centre of mass (--attach-z 0) and the rods'
drone ends at the drone centres (--pivot-dz 0).

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
    # plates 1/3/6/9: the rig layout r0013 capsized on (soft-detach card, 7 Oct)
    'four_rigid_ground_rig_3969.sdf': ['--n', '4', '--azimuths', '30,90,180,270'],
    # Tier B (card 2026-10-16_tier_b_support_margin): drone 1 at 90 leaves, survivors 30 / S / 270, so the
    # survivors' largest gap is S - 30: 120 = _3915, 135, 150 = _3969, 165, 180 (165 and 195 are twin-only azimuths)
    'four_rigid_ground_rig_gap135.sdf': ['--n', '4', '--azimuths', '30,90,165,270'],
    'four_rigid_ground_rig_gap165.sdf': ['--n', '4', '--azimuths', '30,90,195,270'],
    'four_rigid_ground_rig_gap180.sdf': ['--n', '4', '--azimuths', '30,90,210,270'],
    'three_rigid_ground_rig.sdf': ['--n', '3'],
    # ring-mass mismatch neighbours: the planner keeps 0.86 (card A, rig_twin_hover_mismatch*)
    'four_rigid_ground_rig_m070.sdf': ['--n', '4', '--payload-mass', '0.70'],
    'four_rigid_ground_rig_m100.sdf': ['--n', '4', '--payload-mass', '1.0'],
    # 1 m cables (the supervisor's hardware for the week of 19 Oct; plan 2026-10-15 A1)
    'three_rigid_ground_rig_L100.sdf': ['--n', '3'],
    # thesis baseline and the 90 deg chain (plan 2026-10-21): two quads on plates 0/6, and the T on plates 0/3/6
    # (generator azimuths count counter-clockwise, so clockwise plate 3 is 270)
    'two_rigid_ground_rig.sdf': ['--n', '2'],
    'three_rigid_ground_rig_T.sdf': ['--n', '3', '--azimuths', '0,270,180'],
    # ... and the same at 1 m: the thesis baseline B and the chain N (every link a ~1 m cable from 22 Oct)
    'two_rigid_ground_rig_L100.sdf': ['--n', '2'],
    'four_rigid_ground_rig_L100.sdf': ['--n', '4'],
    'three_rigid_ground_rig_T_L100.sdf': ['--n', '3', '--azimuths', '0,270,180'],
    # the M1 layout at 1 m (Wesley 22 Oct): carriers on plates 3/11/7 and QUAD4 on 9 as the fourth carrier
    'four_rigid_ground_rig_3915_L100.sdf': ['--n', '4', '--azimuths', '30,90,150,270'],
    # scalability S (card 2026-10-23_scalability, Wesley 22 Oct): 8 quads on a bigger ring, R 0.5 m (outer 0.53),
    # 0.9 kg, a plate under every quad; 6 quads on it as the neighbour, and 9 on R 0.53 m as the extra hover
    'eight_rigid_ground_rig_R50_L100.sdf': ['--n', '8', '--payload-mass', '0.9', '--plates', '8'],
    'six_rigid_ground_rig_R50_L100.sdf': ['--n', '6', '--payload-mass', '0.9', '--plates', '6'],
    'nine_rigid_ground_rig_R53_L100.sdf': ['--n', '9', '--payload-mass', '0.9', '--plates', '9'],
    # thesis R (card 2026-10-23_refly_1m; Wesley 23 Oct): Tier B's layouts and the push world at 1 m
    'four_rigid_ground_rig_3969_L100.sdf': ['--n', '4', '--azimuths', '30,90,180,270'],
    'four_rigid_ground_rig_gap135_L100.sdf': ['--n', '4', '--azimuths', '30,90,165,270'],
    'four_rigid_ground_rig_gap165_L100.sdf': ['--n', '4', '--azimuths', '30,90,195,270'],
    'four_rigid_ground_rig_gap180_L100.sdf': ['--n', '4', '--azimuths', '30,90,210,270'],
    'three_rigid_ground_rig_push_L100.sdf': ['--n', '3'],
    # the rejoin leaning-pair test (card 2026-10-23_rejoin_departed; Wesley 23 Oct ~08:00): the T at 1 m with the push
    # tool, a steady torque on the ring holding the pair +-3 deg toward the rejoin plate
    'three_rigid_ground_rig_T_push_L100.sdf': ['--n', '3', '--azimuths', '0,270,180'],
    # thesis C on the paper's own geometry (card 2026-10-23_paper_method_compare; Wesley 23 Oct): the 1 m four with
    # every rod on a 1 mm hub at the ring's centre
    'four_rigid_ground_rig_hub_L100.sdf': ['--n', '4', '--hub-r', '0.001'],
    # Quan et al. exactly, world C (decisions 2026-10-23, Wesley ~21:45; card 2026-10-23_quan_exact): the hub at the
    # ring's centre of mass and every rod's other end at the drone's centre, the paper's model to the letter
    'four_rigid_ground_rig_hubcom_L100.sdf': ['--n', '4', '--hub-r', '0.001', '--attach-z', '0.0'],
}
# worlds that also carry the runner's WRENCH tool, as the 0.55 m three/four_rigid_ground_rig_push.sdf
PUSH_WORLDS = ('three_rigid_ground_rig_push_L100.sdf', 'three_rigid_ground_rig_T_push_L100.sdf')
USER_COMMANDS = ('    <plugin filename="ignition-gazebo-user-commands-system" '
                 'name="gz::sim::systems::UserCommands"/>\n')
WRENCH_PLUGIN = ('    <!-- W3 push tool (card 2026-10-03_xy_stiffness_b3): runner WRENCH -> '
                 '/world/<name>/wrench/persistent -->\n'
                 '    <plugin filename="gz-sim-apply-link-wrench-system" '
                 'name="gz::sim::systems::ApplyLinkWrench"/>\n')
# per-world options that override RIG_ARGS (passed after them)
WORLD_OVERRIDES = {w: ['--cable-len', '1.0'] for w in WORLDS if w.endswith('_L100.sdf')}
# the S rings: 8 quads 0.765 m apart (chord) at 60 deg on 1 m cables; real time capped at 0.4, where an 8-quad planner
# tick (100-220 ms wall in the twin) stays inside its 100 ms of sim
for w, (r, outer) in {'eight_rigid_ground_rig_R50_L100.sdf': (0.5, 0.53),
                      'six_rigid_ground_rig_R50_L100.sdf': (0.5, 0.53),
                      'nine_rigid_ground_rig_R53_L100.sdf': (0.53, 0.56)}.items():
    WORLD_OVERRIDES[w] += ['--attach-radius', str(r), '--ring-outer-r', str(outer), '--real-time-factor', '0.4',
                           '--tether-pose-hz', '60']
# the baseline and chain worlds publish each rod's pose, which the mocap emulator turns into the rig's magnet marker
# (card 2026-10-21_tip_detect; 60 Hz keeps the planner's callbacks light; in the existing worlds, Wesley 21 Oct)
for w in ('four_rigid_ground_rig.sdf', 'three_rigid_ground_rig_T.sdf', 'two_rigid_ground_rig.sdf',
          'three_rigid_ground_rig.sdf', 'four_rigid_ground_rig_L100.sdf', 'three_rigid_ground_rig_T_L100.sdf',
          'two_rigid_ground_rig_L100.sdf', 'three_rigid_ground_rig_L100.sdf', 'four_rigid_ground_rig_3915_L100.sdf',
          'four_rigid_ground_rig_3969_L100.sdf', 'four_rigid_ground_rig_gap135_L100.sdf',
          'four_rigid_ground_rig_gap165_L100.sdf', 'four_rigid_ground_rig_gap180_L100.sdf',
          'three_rigid_ground_rig_push_L100.sdf', 'three_rigid_ground_rig_T_push_L100.sdf',
          'four_rigid_ground_rig_hub_L100.sdf', 'four_rigid_ground_rig_hubcom_L100.sdf'):
    WORLD_OVERRIDES[w] = WORLD_OVERRIDES.get(w, []) + ['--tether-pose-hz', '60']
# world C's rods end at the drone centre (after RIG_ARGS' 4 cm pivot)
WORLD_OVERRIDES['four_rigid_ground_rig_hubcom_L100.sdf'] += ['--pivot-dz', '0.0']
RIG_ARGS = ['--cable-len', str(CABLE_LEN), '--detachable', '--ground-start',
            '--attach-radius', str(ATTACH_RADIUS), '--pivot-dz', str(PIVOT_DZ),
            '--rod-mass', str(ROD_MASS), '--magnet-mass', str(MAGNET_MASS),
            '--ring-outer-r', str(RING_OUTER_R), '--ring-thickness', str(RING_THICKNESS),
            '--drone-model', 'x3_rig']
LEGACY_MODELS = 4               # models/x3_drone0..3.sdf; above them a model is drone 0's, renamed


def _arg(name, flag, default):
    """A world's generator option as flown: its WORLD_OVERRIDES over its WORLDS args over RIG_ARGS."""
    for args in (WORLD_OVERRIDES.get(name, []), WORLDS[name], RIG_ARGS):
        if flag in args:
            return args[args.index(flag) + 1]
    return default


N_MODELS = max(int(_arg(w, '--n', 3)) for w in WORLDS)


def world_expect(name):
    """What check_geometry.world_geometry must read back from a rig world: EXPECT with its own mass, cable, attach
    radius (its hub's, if the rods hang from one) and fleet size."""
    hub = _arg(name, '--hub-r', None)
    return dict(EXPECT, load_mass=float(_arg(name, '--payload-mass', 0.86)),
                cable_len=float(_arg(name, '--cable-len', CABLE_LEN)),
                attach_radius=float(hub if hub is not None else _arg(name, '--attach-radius', ATTACH_RADIUS)),
                attach_z=float(_arg(name, '--attach-z', EXPECT['attach_z'])),
                pivot_dz=float(_arg(name, '--pivot-dz', PIVOT_DZ)),
                num_drones=int(_arg(name, '--n', 3)))


def expected_inertia(name):
    """(Ixx, Izz) of a rig world's ring about its CoG, as the generator writes it (an annulus RING_SEG_RADIAL wide)."""
    sys.path.insert(0, ASSETS)
    from generate_rigid_world import ring_inertia, ring_spec
    ring = ring_spec(float(_arg(name, '--ring-outer-r', RING_OUTER_R)), RING_THICKNESS,
                     float(_arg(name, '--attach-radius', ATTACH_RADIUS)))
    return ring_inertia(ring, float(_arg(name, '--payload-mass', 0.86)))


def legacy_model(i):
    """models/x3_drone{i}.sdf as text; above the four that exist, x3_drone0.sdf with every x3_drone0 renamed (the
    model name, its IMU sensor and topic, the four robotNamespace tags)."""
    if i < LEGACY_MODELS:
        with open(os.path.join(ASSETS, 'models', f'x3_drone{i}.sdf')) as f:
            return f.read()
    with open(os.path.join(ASSETS, 'models', 'x3_drone0.sdf')) as f:
        return f.read().replace('x3_drone0', f'x3_drone{i}')


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
    src = f'x3_drone{i}.sdf' if i < LEGACY_MODELS else f'x3_drone0.sdf renamed x3_drone{i}'
    note = (f'\n    <!-- Rig twin of {src} (tools/make_carry_worlds.py): airframe with '
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
    for i in range(N_MODELS):
        text = rig_model(legacy_model(i), i)
        rel = os.path.join('models', f'x3_rig_drone{i}.sdf')
        with open(os.path.join(out_dir, rel), 'w') as f:
            f.write(text)
        written.append(rel)
    for name, args in WORLDS.items():
        path = os.path.join(out_dir, name)
        subprocess.run([sys.executable, GEN, *args, *RIG_ARGS, *WORLD_OVERRIDES.get(name, []), '--out', path],
                       check=True, cwd=ASSETS, stdout=subprocess.DEVNULL)
        if name in PUSH_WORLDS:
            with open(path) as f:
                text = f.read()
            if text.count(USER_COMMANDS) != 1:
                raise SystemExit(f'{name}: no single UserCommands plugin line to put the WRENCH tool after')
            with open(path, 'w') as f:
                f.write(text.replace(USER_COMMANDS, USER_COMMANDS + WRENCH_PLUGIN))
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
