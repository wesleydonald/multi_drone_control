#!/usr/bin/env python3
"""
tools/make_weld_variants.py
---------------------------
Generate magnet-arm variants of the attach world so the WELD MECHANICS can be varied as
a one-factor experiment instead of by hand-editing a 446-line SDF.

WHY THIS EXISTS

The magnet arm has never been varied: git shows x3_drone3_magnet.sdf was created with
`arm_to_magnet_tip` already a ball joint and touched exactly once since, to change
motorConstant. So "is the weld joint the problem?" has never actually been measured -- it
has only been reasoned about in SDF comments.

Two things are worth knowing, and they bracket the question from opposite sides:

  * the shipped weld is ALREADY fully articulated. The DetachableJoint welds
    magnet_tip_link to the payload, and `arm_to_magnet_tip` is a ball AT THAT POINT, so
    the chain payload=weld=tip-BALL-arm-BALL-drone transmits tension only. It is
    kinematically the same as a tether (payload-BALL-stub=weld=rod-BALL-drone). Adding
    compliance has little room to help; that is what `seg2` tests.

  * the thesis claims sub-claim N3 under a RIGID, moment-transmitting joint. The
    simulator does not currently have one. `rigid` builds it, and is the harder case.

THE LADDER (one factor per step, so a difference is attributable)

  rod    - baseline topology, arm modelled as a proper slender rod. The shipped arm puts
           its whole 1 g at the link origin (the TOP) with isotropic 2e-6, where the
           tether rods use m*L^2/12 = 2.083e-05 about their midpoint. Same joints, so
           this isolates the inertia defect.
  seg2   - `rod` with the arm split into two 0.25 m segments and a third ball joint
           between them. Wesley's hypothesis: more compliance in the arm.
  rigid  - `rod` with `arm_to_magnet_tip` FIXED. The weld now transmits moment into the
           payload -- the configuration the thesis actually claims.

LINK ORDER MATTERS. gz's PosePublisher emits links in REVERSE declaration order, and
magnet_tip_publisher / the mocap emulator address that PoseArray positionally
(magnet_index 0 = the tip, drone_index -1 = the model world pose). Every variant
therefore keeps magnet_tip_link declared LAST, so both ends of the array stay put even
when `seg2` inserts a link in the middle.

Usage:  python3 tools/make_weld_variants.py [--check]

Writes simulation_assets/models/x3_drone3_magnet_<v>.sdf and the matching world
simulation_assets/three_attach_<v>.sdf. Both are GENERATED -- edit this script, not them.
`--check` regenerates into memory and fails if anything on disk is stale.
"""
from __future__ import annotations

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASSETS = os.path.join(REPO, 'simulation_assets')
BASE_MODEL = os.path.join(ASSETS, 'models', 'x3_drone3_magnet.sdf')
BASE_WORLD = os.path.join(ASSETS, 'three_attach.sdf')

# The arm block is delimited by these two comments in the base model. Both are load-bearing
# markers: if either is reworded, this script must fail loudly rather than emit half a model.
BLOCK_START = '<!-- ===== MAGNET ARM:'
BLOCK_END = "<!-- Publish this drone's poses"

ARM_LEN = 0.5        # drone pivot -> magnet tip, matches the fleet cable_len
ARM_MASS = 0.001     # as shipped
TIP_MASS = 0.002


def rod_inertia(mass, length):
    """Slender rod about its own midpoint, spun about the two transverse axes. The axial
    term is the tether rods' 8e-09 -- a 4 mm cylinder has no meaningful polar inertia and a
    zero there makes the solver's mass matrix singular."""
    return mass * length * length / 12.0, 8.0e-09


def arm_link(name, parent, pose, length, mass):
    ixx, izz = rod_inertia(mass, length)
    return f"""        <link name="{name}">
            <pose relative_to="{parent}">{pose}</pose>
            <inertial>
                <pose>0 0 {-0.5 * length:.4f} 0 0 0</pose>
                <mass>{mass}</mass>
                <inertia><ixx>{ixx:.4e}</ixx><ixy>0</ixy><ixz>0</ixz>"""\
        f"""<iyy>{ixx:.4e}</iyy><iyz>0</iyz><izz>{izz:.4e}</izz></inertia>
            </inertial>
            <visual name="{name}_visual">
                <pose>0 0 {-0.5 * length:.4f} 0 0 0</pose>
                <geometry><cylinder><radius>0.002</radius><length>{length:.4f}</length></cylinder></geometry>
                <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
            </visual>
        </link>
"""


def tip_link(parent, drop):
    return f"""        <link name="magnet_tip_link">
            <pose relative_to="{parent}">0 0 {-drop:.4f} 0 0 0</pose>
            <inertial>
                <mass>{TIP_MASS}</mass>
                <inertia><ixx>1e-6</ixx><ixy>0</ixy><ixz>0</ixz><iyy>1e-6</iyy><iyz>0</iyz><izz>1e-6</izz></inertia>
            </inertial>
            <visual name="magnet_tip_visual">
                <geometry><sphere><radius>0.025</radius></sphere></geometry>
                <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse><specular>0.2 0.2 0.2 1</specular></material>
            </visual>
        </link>
"""


def ball(name, parent, child, pose=None):
    pose_tag = f'\n            <pose>{pose}</pose>' if pose else ''
    return f"""        <joint name="{name}" type="ball">
            <parent>{parent}</parent>
            <child>{child}</child>{pose_tag}
            <axis><xyz>1 0 0</xyz><dynamics><damping>0.1</damping></dynamics></axis>
            <axis2><xyz>0 1 0</xyz><dynamics><damping>0.1</damping></dynamics></axis2>
        </joint>
"""


def fixed(name, parent, child):
    return f"""        <joint name="{name}" type="fixed">
            <parent>{parent}</parent>
            <child>{child}</child>
        </joint>
"""


def block_rod(header):
    return (header
            + arm_link('magnet_arm', 'base_link', '0 0 0.01 0 0 0', ARM_LEN, ARM_MASS)
            + tip_link('magnet_arm', ARM_LEN)
            + ball('arm_to_magnet_tip', 'magnet_arm', 'magnet_tip_link')
            + ball('base_to_magnet_arm', 'base_link', 'magnet_arm', '0 0 -0.05 0 0 0'))


def block_seg2(header):
    half = ARM_LEN / 2.0
    return (header
            + arm_link('magnet_arm', 'base_link', '0 0 0.01 0 0 0', half, ARM_MASS / 2)
            + arm_link('magnet_arm_lower', 'magnet_arm',
                       f'0 0 {-half:.4f} 0 0 0', half, ARM_MASS / 2)
            + tip_link('magnet_arm_lower', half)
            + ball('arm_to_magnet_tip', 'magnet_arm_lower', 'magnet_tip_link')
            + ball('arm_mid', 'magnet_arm', 'magnet_arm_lower')
            + ball('base_to_magnet_arm', 'base_link', 'magnet_arm', '0 0 -0.05 0 0 0'))


def block_rigid(header):
    return (header
            + arm_link('magnet_arm', 'base_link', '0 0 0.01 0 0 0', ARM_LEN, ARM_MASS)
            + tip_link('magnet_arm', ARM_LEN)
            + fixed('arm_to_magnet_tip', 'magnet_arm', 'magnet_tip_link')
            + ball('base_to_magnet_arm', 'base_link', 'magnet_arm', '0 0 -0.05 0 0 0'))


HEADERS = {
    'rod': """        <!-- ===== MAGNET ARM (variant 'rod', GENERATED by tools/make_weld_variants.py)
             Baseline topology with the arm modelled as a proper slender rod: mass at the
             MIDPOINT with m*L^2/12, matching the tether rods. The shipped model puts the
             whole 1 g at the link origin (the top) with an isotropic 2e-6, i.e. 10x too
             little transverse inertia in the wrong place. Joints unchanged, so any
             difference from the baseline is the inertia alone. ===== -->
""",
    'seg2': """        <!-- ===== MAGNET ARM (variant 'seg2', GENERATED by tools/make_weld_variants.py)
             'rod' with the arm split into two 0.25 m segments and a THIRD ball joint
             between them, so the tether can bend mid-span instead of only pivoting at its
             ends. Total length, mass and damping are unchanged. ===== -->
""",
    'rigid': """        <!-- ===== MAGNET ARM (variant 'rigid', GENERATED by tools/make_weld_variants.py)
             'rod' with arm_to_magnet_tip FIXED. The tip is already welded rigidly to the
             payload, so making this joint fixed too means the arm transmits a MOMENT into
             the load: the moment-transmitting attachment sub-claim N3 describes, and
             which no shipped world currently contains. Expected to be the hard case. ===== -->
""",
}

BUILDERS = {'rod': block_rod, 'seg2': block_seg2, 'rigid': block_rigid}


def read(path):
    with open(path) as f:
        return f.read()


def build_model(base, variant):
    i = base.find(BLOCK_START)
    j = base.find(BLOCK_END)
    if i < 0 or j < 0 or j <= i:
        raise SystemExit(
            f'{BASE_MODEL}: could not locate the magnet-arm block between\n'
            f'  {BLOCK_START!r}\nand\n  {BLOCK_END!r}\n'
            '-- the base model was reworded; update tools/make_weld_variants.py.')
    indent = base.rfind('\n', 0, i) + 1
    return base[:indent] + BUILDERS[variant](HEADERS[variant]) + base[indent + (j - i):]


def build_world(base, variant):
    src = '<uri>models/x3_drone3_magnet.sdf</uri>'
    if src not in base:
        raise SystemExit(f'{BASE_WORLD}: magnet model include not found ({src})')
    world = base.replace(src, f'<uri>models/x3_drone3_magnet_{variant}.sdf</uri>')
    return world.replace(
        '<sdf version="1.6">',
        f'<sdf version="1.6">\n  <!-- GENERATED from three_attach.sdf by '
        f'tools/make_weld_variants.py (weld variant: {variant}). Do not edit. -->', 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true',
                    help='fail if the generated files on disk are stale')
    args = ap.parse_args()

    model_base, world_base = read(BASE_MODEL), read(BASE_WORLD)
    stale = []
    for variant in BUILDERS:
        for path, text in (
                (os.path.join(ASSETS, 'models', f'x3_drone3_magnet_{variant}.sdf'),
                 build_model(model_base, variant)),
                (os.path.join(ASSETS, f'three_attach_{variant}.sdf'),
                 build_world(world_base, variant))):
            if args.check:
                if not os.path.exists(path) or read(path) != text:
                    stale.append(os.path.relpath(path, REPO))
                continue
            with open(path, 'w') as f:
                f.write(text)
            print(f'  wrote {os.path.relpath(path, REPO)}')

    if args.check:
        if stale:
            print('STALE (re-run tools/make_weld_variants.py):')
            for p in stale:
                print(f'  {p}')
            return 1
        print('  weld variants up to date')
    return 0


if __name__ == '__main__':
    sys.exit(main())
