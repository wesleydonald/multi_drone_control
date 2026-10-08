#!/usr/bin/env python3
"""
tools/make_attach_worlds.py
---------------------------
Generate the rig-twin attach newcomer and its two worlds, so the M1 rejoin rungs fly in
the rig-matched twin instead of the legacy sim (Wesley, 8 Oct 2026; docs/decisions.md):

  models/x3_rig_drone3_magnet.sdf          QUAD4 = x3_drone3: the twin airframe
                                           (x3_rig_drone3, 0.475 kg) + a landing skid + a
                                           magnet rod from a pivot 4 cm below the body centre,
                                           0.55 m to the magnet (0.040 kg rod, 0.035 kg magnet
                                           as the tip mass): 0.55 kg in all, body to tip ~0.59 m.
                                           Arm spawned laid out along body +y on the floor.
  models/x3_rig_drone3_magnet_welded.sdf   the same drone with the arm spawned along body -y at
                                           the carriers' ground-start elevation (a link's spawn
                                           pose cannot be set per <include>, so the welded start
                                           needs its own file, as the legacy partner/_attached pair)
  three_rigid_ground_rig_attach.sdf        FREE start (R8a0/R8a/R8b/R9): the twin ring, three
                                           carriers on the floor at 270/30/150 deg (rig plates
                                           3/11/7), the newcomer on the floor at (0, 1.4), its
                                           magnet on the floor 0.55 m further out (+y)
  three_rigid_ground_rig_attach_welded.sdf WELDED floor start (R10b/R11): the newcomer welded
                                           on rig plate 9 (90 deg, the plate at (0, 0.225)),
                                           its tip on the plate, the arm at the carriers'
                                           ground-start elevation

The carriers and ring are generate_rigid_world.py with make_carry_worlds.RIG_ARGS, i.e. the
same twin as the carry worlds. Rig plate labels run clockwise from +x (plate 9 at 90 deg);
the ring's plate_k_visual are numbered counter-clockwise (plate_3_visual at 90 deg).

The gz DetachableJoint welds at spawn. The free world relies on the magnet manager's
detach_on_start (its default) to free the magnet; the welded world needs attached_at_start.

The magnet has a floor collision so the laid-out arm stays on the floor (the legacy tip had
none and hung through it). Its collide_bitmask 0x02 and the ring's 0x01 keep it out of
contact with the ring, so the weld (tip on the plate) and the approach see no ring contact,
as with the legacy tip.

Usage:  python3 tools/make_attach_worlds.py [--check]

Every file here is GENERATED -- edit this script, not them. `--check` regenerates into a
temp dir and fails if anything on disk is stale.
"""
import argparse
import math
import os
import re
import subprocess
import sys
import tempfile

from make_carry_worlds import (ASSETS, ATTACH_RADIUS, CABLE_LEN, GEN, MAGNET_MASS, PIVOT_DZ,
                               RIG_ARGS, RING_OUTER_R, RING_THICKNESS, ROD_MASS, rig_model)
from make_weld_variants import arm_link

ID = 3
MODEL = 'x3_rig_drone3_magnet'
CARRIER_AZIMUTHS = '270,30,150'   # drone_0/1/2 on rig plates 3/11/7 (decisions 8 Oct)
WELD_Y = ATTACH_RADIUS            # rig plate 9 at 90 deg, (0, +0.225) (attach_y_offset)
FREE_XY = (0.0, 1.4)
GROUND_Z = 0.10                   # every twin drone rests here (--ground-start)
PLATE_Z = RING_THICKNESS          # plate top, ring on the floor (CoG t/2, plane t/2 up)
TIP_R = 0.012                     # the carriers' magnet sphere
SKID_Z = -0.08                    # bottom flush with the airframe box (-0.10)
BODY_DAMPING = 0.002              # universal joints, the legacy newcomer's (decisions 2026-09-27)
TIP_DAMPING = 1e-4
RING_BITMASK = '0x01'
TIP_BITMASK = '0x02'

WORLDS = {
    'three_rigid_ground_rig_attach.sdf': 'free',
    'three_rigid_ground_rig_attach_welded.sdf': 'welded',
}
MODELS = {'free': f'{MODEL}.sdf', 'welded': f'{MODEL}_welded.sdf'}


def _f(x):
    return f'{x + 0.0:.6f}'


def spawn(start):
    """(drone xyz, arm roll in base_link). The arm runs pivot -> tip in the body y-z plane:
    roll r turns its -z axis to (0, sin r, -cos r). Free: the magnet resting on the floor out
    along +y. Welded: the tip on the plate top at (0, WELD_Y), the rod at the carriers'
    ground-start elevation, so the drone sits on the carriers' circle."""
    pivot_z = GROUND_Z - PIVOT_DZ
    if start == 'free':
        dz = pivot_z - TIP_R
        h = math.sqrt(CABLE_LEN ** 2 - dz ** 2)
        return (FREE_XY[0], FREE_XY[1], GROUND_Z), math.atan2(h, dz)
    dz = pivot_z - PLATE_Z
    h = math.sqrt(CABLE_LEN ** 2 - dz ** 2)
    return (0.0, WELD_Y + h, GROUND_Z), -math.atan2(h, dz)


def _universal(name, parent, child, damping, pose=None, note=None):
    tag = f'\n            <pose>{pose}</pose>' if pose else ''
    if note:
        tag += f'<!-- {note} -->'
    return f"""        <joint name="{name}" type="universal">
            <parent>{parent}</parent>
            <child>{child}</child>{tag}
            <axis><xyz>1 0 0</xyz><dynamics><damping>{damping:g}</damping></dynamics></axis>
            <axis2><xyz>0 1 0</xyz><dynamics><damping>{damping:g}</damping></dynamics></axis2>
        </joint>
"""


def _tip_link(parent, drop):
    i = 0.4 * MAGNET_MASS * TIP_R ** 2
    sphere = f'<geometry><sphere><radius>{TIP_R}</radius></sphere></geometry>'
    return f"""        <link name="magnet_tip_link">
            <pose relative_to="{parent}">0 0 {-drop:.4f} 0 0 0</pose>
            <inertial>
                <mass>{MAGNET_MASS}</mass>
                <inertia><ixx>{i:.4e}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{i:.4e}</iyy><iyz>0</iyz><izz>{i:.4e}</izz></inertia>
            </inertial>
            <collision name="magnet_tip_collision">
                {sphere}
                <surface><contact><collide_bitmask>{TIP_BITMASK}</collide_bitmask></contact></surface>
            </collision>
            <visual name="magnet_tip_visual">
                {sphere}
                <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse><specular>0.2 0.2 0.2 1</specular></material>
            </visual>
        </link>
"""


SKID = f"""            <!-- Wide, low landing skid so the free newcomer rests flat on the floor (no tether
                 holds it up). Contact only, no mass. Its bottom is flush with the airframe box,
                 so the newcomer rests at z {GROUND_Z} like the twin carriers (legacy: -0.12). -->
            <collision name="landing_skid_collision">
                <pose frame="">0 0 {SKID_Z} 0 0 0</pose>
                <geometry>
                    <box>
                        <size>0.3 0.3 0.04</size>
                    </box>
                </geometry>
            </collision>
"""

NESTED_POSE_PUBLISHER = """        <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
            <publish_link_pose>true</publish_link_pose>
            <publish_nested_model_pose>true</publish_nested_model_pose>
            <use_pose_vector_msg>true</use_pose_vector_msg>
            <update_frequency>500</update_frequency>
        </plugin>

"""

TAIL = """        <!-- A top-level model: publish_model_pose adds the model's WORLD pose (the body) as the
             last PoseArray entry on /model/x3_drone3/pose; the link poses are model-relative.
             magnet_tip_link is declared last, so it is entry 0 (magnet_tip_publisher
             magnet_index 0, drone_index -1; mocap pose_index -1). -->
        <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
            <publish_model_pose>true</publish_model_pose>
            <publish_link_pose>true</publish_link_pose>
            <publish_nested_model_pose>false</publish_nested_model_pose>
            <use_pose_vector_msg>true</use_pose_vector_msg>
            <update_frequency>500</update_frequency>
        </plugin>

        <!-- MAGNET WELD: /payload/attach joins the magnet tip rigidly to the shared ring
             (lift_system::payload), /payload/detach releases it. gz creates it at spawn. -->
        <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
            <parent_link>magnet_tip_link</parent_link>
            <child_model>lift_system::payload</child_model>
            <child_link>body</child_link>
            <detach_topic>/payload/detach</detach_topic>
            <attach_topic>/payload/attach</attach_topic>
            <output_topic>/payload/detachable_joint_state</output_topic>
        </plugin>
"""


def _replace_once(text, old, new, what):
    if text.count(old) != 1:
        raise SystemExit(f'x3_rig_drone{ID}: {what} not found exactly once; update '
                         'tools/make_attach_worlds.py')
    return text.replace(old, new, 1)


def newcomer_model(start):
    """x3_rig_drone3 (make_carry_worlds.rig_model) + skid + magnet arm, its pose publisher
    swapped for the free drone's (model world pose) and the ring weld added."""
    with open(os.path.join(ASSETS, 'models', f'x3_drone{ID}.sdf')) as f:
        text = rig_model(f.read(), ID)
    _, roll = spawn(start)
    hdr = '<sdf version="1.6">'
    text = _replace_once(text, hdr, hdr + (
        f'\n    <!-- Rig-twin attach newcomer (QUAD4), GENERATED by tools/make_attach_worlds.py: '
        f'do not edit.\n         {start} start: the arm spawns '
        + ('laid out along body +y with the magnet on the floor. -->' if start == 'free' else
           'along body -y at the carriers\' ground-start elevation, tip on the plate. -->')),
        'header')
    box = '<collision name="base_link_inertia_collision">'
    end = text.index('</collision>\n', text.index(box)) + len('</collision>\n')
    text = text[:end] + SKID + text[end:]
    text = _replace_once(text, NESTED_POSE_PUBLISHER, '', 'airframe pose publisher')
    arm = (f"""
        <!-- ===== MAGNET ARM (rig twin) =====
             As the twin carriers' rods: a pivot {PIVOT_DZ} m below the body centre, {CABLE_LEN} m to the
             magnet (QUAD4 0.541-0.546 m pivot-to-plate, 8 Oct), {ROD_MASS} kg along the rod and the
             {MAGNET_MASS} kg magnet as the tip mass. Universal joints at both ends with the legacy
             newcomer's damping (DART ignores ball damping; rod twist blocked, decisions 2026-09-27). -->
"""
           + arm_link('magnet_arm', 'base_link', f'0 0 {-PIVOT_DZ:.4f} {roll:.6f} 0 0',
                      CABLE_LEN, ROD_MASS)
           + _tip_link('magnet_arm', CABLE_LEN)
           + _universal('arm_to_magnet_tip', 'magnet_arm', 'magnet_tip_link', TIP_DAMPING)
           + _universal('base_to_magnet_arm', 'base_link', 'magnet_arm', BODY_DAMPING,
                        '0 0 0 0 0 0', note=f'the pivot: the arm origin, {PIVOT_DZ} m below '
                                            'the body centre')
           + TAIL)
    close = '    </model>\n</sdf>'
    i = text.rfind(close)
    if i < 0:
        raise SystemExit(f'x3_rig_drone{ID}: closing </model> not found')
    return text[:i] + arm + text[i:]


def carrier_world():
    """The three-carrier twin on the M1 layout: the carry worlds' generator call."""
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, 'w.sdf')
        subprocess.run([sys.executable, GEN, '--n', '3', '--azimuths', CARRIER_AZIMUTHS,
                        *RIG_ARGS, '--out', out],
                       check=True, cwd=ASSETS, stdout=subprocess.DEVNULL)
        with open(out) as f:
            return f.read()


RING_COLLISION = (f'<geometry><cylinder><radius>{RING_OUTER_R:.3f}</radius><length>'
                  f'{RING_THICKNESS:.3f}</length></cylinder></geometry>\n          </collision>')


def attach_world(base, start):
    (x, y, z), _ = spawn(start)
    carrier = re.search(r'<name>x3_drone0</name>\s*<pose>(\S+) (\S+) (\S+)', base)
    if start == 'welded' and (
            abs(math.hypot(float(carrier[1]), float(carrier[2])) - y) > 1e-5
            or abs(float(carrier[3]) - z) > 1e-9):
        raise SystemExit("welded newcomer is off the carriers' ground-start circle")
    text = _replace_once(base, '<?xml version="1.0" ?>\n', '<?xml version="1.0" ?>\n' + (
        '<!-- GENERATED by tools/make_attach_worlds.py: do not edit. Rig-twin M1 attach world, '
        f'{start} start: carriers on the floor at {CARRIER_AZIMUTHS.replace(",", "/")} deg '
        '(rig plates 3/11/7), newcomer x3_drone3 '
        + ('free on the floor at (0, 1.4), magnet on the floor further out (+y). -->\n'
           if start == 'free' else
           'welded on rig plate 9 (90 deg, the plate at (0, 0.225)) from spawn. -->\n')),
        'xml declaration')
    text = _replace_once(text, RING_COLLISION, RING_COLLISION.replace(
        '</geometry>', '</geometry>\n            <surface><contact><collide_bitmask>'
        f'{RING_BITMASK}</collide_bitmask></contact></surface><!-- not the newcomer\'s '
        f'magnet ({TIP_BITMASK}) -->', 1), 'ring collision')
    where = ('Free start: on the floor at (0, 1.4), the arm laid out along +y with the magnet\n'
             '         on the floor; the magnet manager frees the spawn weld (detach_on_start).'
             if start == 'free' else
             'Welded start: welded on rig plate 9 (90 deg) from spawn, the tip on the plate top\n'
             '         and the rod at the carriers\' ground-start elevation; the magnet manager\n'
             '         starts attached (attached_at_start).')
    block = f"""    <!-- ===== NEWCOMER (QUAD4 = /drone_3): a free rig-twin drone with a {CABLE_LEN} m magnet rod =====
         Standalone, not in lift_system: its poses are on /model/x3_drone3/pose.
         {where} -->
    <include>
      <uri>models/{MODELS[start]}</uri>
      <name>x3_drone{ID}</name>
      <pose>{_f(x)} {_f(y)} {_f(z)} 0 0 0</pose>
    </include>
"""
    return _replace_once(text, '  </world>\n</sdf>', block + '  </world>\n</sdf>', 'world end')


def generate(out_dir):
    """Write the newcomer models and both worlds under out_dir (same layout as ASSETS)."""
    os.makedirs(os.path.join(out_dir, 'models'), exist_ok=True)
    files = {os.path.join('models', name): newcomer_model(start)
             for start, name in MODELS.items()}
    base = carrier_world()
    files.update({name: attach_world(base, start) for name, start in WORLDS.items()})
    for rel, text in files.items():
        with open(os.path.join(out_dir, rel), 'w') as f:
            f.write(text)
    return list(files)


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
        print('rerun tools/make_attach_worlds.py')
        return 1
    print('rig attach worlds and newcomer models up to date')
    return 0


if __name__ == '__main__':
    sys.exit(main())
