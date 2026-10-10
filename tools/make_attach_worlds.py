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
  models/x3_rig_drone3_magnet_pickup.sdf   the free newcomer + the partner model's second weld
                                           (/pickup/*) onto Tejen's ball (tejen/payload_model.sdf)
  models/rig_pickup_ball.sdf               Tejen's ball (tejen/payload_model.sdf) on collide bits
                                           0x05, so QUAD4's magnet (0x02) passes through it
  three_rigid_ground_rig_attach_pickup.sdf the combined M1 subtest (Wesley 10 Oct): the free world
                                           with the pickup newcomer, an open ring (annulus + the
                                           partner world's net, scaled) and Tejen's ball on the
                                           floor; his controller takes QUAD4 off, picks the ball
                                           up, drops it into the net, then attaches on plate 9
  models/x3_rig_drone3_magnet_welded_pickup.sdf
                                           the welded newcomer + the same pickup weld
  three_rigid_ground_rig_attach_welded_pickup.sdf
                                           the full M1 (Wesley 10 Oct): the welded world with the
                                           welded pickup newcomer, the open ring with its net and
                                           Tejen's ball; QUAD4 lifts the ring as the fourth carrier,
                                           detaches, is released to his controller (pickup, drop),
                                           then attaches on plate 9 again
  three_rigid_ground_rig_attach_push.sdf   the free world + the runner's WRENCH tool (ApplyLinkWrench)
  three_rigid_ground_rig_attach_pickup_push.sdf
                                           the pickup world + the same tool (the combined M1
                                           card's _abort arm: a push on the ring)
  three_rigid_ground_rig_attach_welded_pickup_push.sdf
                                           the full M1 world + the same tool (the tilt cut card's
                                           push on QUAD4)

The carriers and ring are generate_rigid_world.py with make_carry_worlds.RIG_ARGS, i.e. the
same twin as the carry worlds. Rig plate labels run clockwise from +x (plate 9 at 90 deg);
the ring's plate_k_visual are numbered counter-clockwise (plate_3_visual at 90 deg).

The gz DetachableJoint welds at spawn. The free world relies on the magnet manager's
detach_on_start (its default) to free the magnet; the welded world needs attached_at_start.

The magnet has a floor collision so the laid-out arm stays on the floor (the legacy tip had
none and hung through it). Its collide_bitmask 0x02 and the ring's 0x01 keep it out of
contact with the ring, so the weld (tip on the plate) and the approach see no ring contact,
as with the legacy tip.

In the pickup world the net has its own bit, 0x04: the ball lands in it, the magnet (0x02) and the
floor pass through it. The floor there is 0x03 (the ring and the magnet still rest on it), so the
net may sag below the ring's resting height, as in the partner world. The ball is a copy on 0x05:
the floor, the ring and the net, but not the magnet, which in the legacy world had no collision
at all (card 2026-10-10_combined_m1_contact, critic round 3).

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
import xml.etree.ElementTree as ET

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
NET_BITMASK = '0x04'              # pickup world: the net catches the ball only
PICKUP_FLOOR_BITMASK = '0x03'     # the ring and the magnet rest on the floor, the net passes

# Combined M1 subtest (Wesley 10 Oct): Tejen's ball and the pickup weld of his partner model
PICKUP_MODEL = f'{MODEL}_pickup.sdf'
PICKUP_WORLD = 'three_rigid_ground_rig_attach_pickup.sdf'
# the full M1 (Wesley 10 Oct): the same with QUAD4 welded on plate 9 from spawn
PICKUP_MODELS = {'free': PICKUP_MODEL, 'welded': f'{MODEL}_welded_pickup.sdf'}
PICKUP_WORLDS = {'free': PICKUP_WORLD, 'welded': 'three_rigid_ground_rig_attach_welded_pickup.sdf'}
BALL_SDF = 'tejen/payload_model.sdf'
BALL_MODEL, BALL_LINK = 'payload_model', 'payload_link'
BALL_COPY = 'rig_pickup_ball.sdf'   # under models/
BALL_BITMASK = '0x05'             # the floor (0x03), the ring (0x01), the net (0x04); not the tip (0x02)
BALL_XY = (1.0, 1.6)              # (1.0, 1.4) was 1.06 m from carrier 1 at (0.67, 0.39)
BALL_CLEAR_CARRIER = 1.2          # m, ball centre to each carrier's centre
BALL_CLEAR_NEWCOMER = 1.0         # m, ball centre to QUAD4's body and arm (laid out, or welded on plate 9)
# the rig ring is open: its 60 mm rim centred on the plates, as ring_k_visual draws it
RING_INNER_R = 2 * ATTACH_RADIUS - RING_OUTER_R
ANNULUS_SEGMENTS = 16             # the partner world's; each box on the floor costs RTF
# the partner world's net (old_worlds/three_attach_partner_attached.sdf; R0625: the ball rolled
# off the solid disc): rim at its annulus's inner radius, heights below the ring's top face
LEGACY_NET_RIM_R = 0.20
NET_PANELS = 16
NET_PANEL_R, NET_PANEL_DZ = 0.125, -0.042
NET_PANEL_LEN, NET_PANEL_W, NET_PITCH = 0.174, 0.0836, 0.49
NET_CENTRE_R, NET_CENTRE_DZ = 0.055, -0.082
NET_THICK, NET_VIS_THICK = 0.004, 0.002

WORLDS = {
    'three_rigid_ground_rig_attach.sdf': 'free',
    'three_rigid_ground_rig_attach_welded.sdf': 'welded',
}
MODELS = {'free': f'{MODEL}.sdf', 'welded': f'{MODEL}_welded.sdf'}
# a world with the runner's WRENCH tool -> the world it copies: the free start (a push on the newcomer:
# its envelope fault, card 2026-10-09_newcomer_fault_scope; Wesley 8 Oct night) and the pickup world
# (a push on the ring during his descent: the combined M1 card's _abort arm)
PUSH_WORLDS = {'three_rigid_ground_rig_attach_push.sdf': 'three_rigid_ground_rig_attach.sdf',
               'three_rigid_ground_rig_attach_pickup_push.sdf': PICKUP_WORLD,
               # the full M1 + the WRENCH tool (card 2026-10-11_quad4_tilt_cut: a push on QUAD4 in his flight)
               'three_rigid_ground_rig_attach_welded_pickup_push.sdf': PICKUP_WORLDS['welded']}
USER_COMMANDS = ('    <plugin filename="ignition-gazebo-user-commands-system" '
                 'name="gz::sim::systems::UserCommands"/>\n')
WRENCH_PLUGIN = ('    <!-- push tool: runner WRENCH -> /world/<name>/wrench/persistent -->\n'
                 '    <plugin filename="gz-sim-apply-link-wrench-system" '
                 'name="gz::sim::systems::ApplyLinkWrench"/>\n')


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
RING_COLLISION_MASKED = RING_COLLISION.replace(
    '</geometry>', '</geometry>\n            <surface><contact><collide_bitmask>'
    f'{RING_BITMASK}</collide_bitmask></contact></surface><!-- not the newcomer\'s '
    f'magnet ({TIP_BITMASK}) -->', 1)
XML_DECL = '<?xml version="1.0" ?>\n'


def _header(start):
    return ('<!-- GENERATED by tools/make_attach_worlds.py: do not edit. Rig-twin M1 attach world, '
            f'{start} start: carriers on the floor at {CARRIER_AZIMUTHS.replace(",", "/")} deg '
            '(rig plates 3/11/7), newcomer x3_drone3 '
            + ('free on the floor at (0, 1.4), magnet on the floor further out (+y). -->\n'
               if start == 'free' else
               'welded on rig plate 9 (90 deg, the plate at (0, 0.225)) from spawn. -->\n'))


def attach_world(base, start):
    (x, y, z), _ = spawn(start)
    carrier = re.search(r'<name>x3_drone0</name>\s*<pose>(\S+) (\S+) (\S+)', base)
    if start == 'welded' and (
            abs(math.hypot(float(carrier[1]), float(carrier[2])) - y) > 1e-5
            or abs(float(carrier[3]) - z) > 1e-9):
        raise SystemExit("welded newcomer is off the carriers' ground-start circle")
    text = _replace_once(base, XML_DECL, XML_DECL + _header(start), 'xml declaration')
    text = _replace_once(text, RING_COLLISION, RING_COLLISION_MASKED, 'ring collision')
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


PICKUP_WELD = f"""
        <!-- PICKUP WELD (Tejen's mission, as the partner model): the same magnet tip grabs his ball
             ({BALL_SDF}). Separate topics so the ring weld and the ball weld never collide.
             gz creates it at spawn too; partner_mission.launch.py frees it (/pickup/detach). -->
        <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">
            <parent_link>magnet_tip_link</parent_link>
            <child_model>{BALL_MODEL}</child_model>
            <child_link>{BALL_LINK}</child_link>
            <detach_topic>/pickup/detach</detach_topic>
            <attach_topic>/pickup/attach</attach_topic>
            <output_topic>/pickup/detachable_joint_state</output_topic>
        </plugin>
"""

# the partner world's block, so /model/payload_model/pose carries [link, model] as there
BALL_POSE_PUBLISHER = """      <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <publish_model_pose>true</publish_model_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <update_frequency>500</update_frequency>
      </plugin>
"""

FLOOR_COLLISION = ('<collision name="collision"><geometry><plane><normal>0 0 1</normal>'
                   '<size>100 100</size></plane></geometry></collision>')
NET_LOOK = ('<material><ambient>0.9 0.9 0.9 0.35</ambient><diffuse>0.9 0.9 0.9 0.35</diffuse>'
            '</material><transparency>0.6</transparency>')


BALL_COLLISION_END = '        </geometry>\n      </collision>'


def ball_model():
    """Tejen's ball with its collision on BALL_BITMASK. The model and link names stay: the pickup
    weld and his launch's /model/payload_model/pose bridge use them."""
    with open(os.path.join(ASSETS, BALL_SDF)) as f:
        text = f.read()
    hdr = '<sdf version="1.6">'
    for old, what in ((hdr, 'the sdf tag'), (BALL_COLLISION_END, 'the collision end')):
        if text.count(old) != 1:
            raise SystemExit(f'{BALL_SDF}: {what} not found exactly once; update '
                             'tools/make_attach_worlds.py')
    text = text.replace(hdr, hdr + (
        f'\n  <!-- GENERATED by tools/make_attach_worlds.py from {BALL_SDF}: do not edit. Its collision '
        f'is on bits {BALL_BITMASK}:\n       the floor, the ring ({RING_BITMASK}) and the net '
        f'({NET_BITMASK}), not QUAD4\'s magnet ({TIP_BITMASK}). -->'), 1)
    return text.replace(BALL_COLLISION_END, (
        '        </geometry>\n        <surface><contact><collide_bitmask>'
        f'{BALL_BITMASK}</collide_bitmask></contact></surface>\n      </collision>'), 1)


def ball(text):
    """The ball copy: (resting height, collide bitmask). Fails if its model or link is not what
    the pickup weld names, or if it would miss the floor, the ring or the net, or hit the tip."""
    model = ET.fromstring(text).find('model')
    links = model.findall('link')
    if model.get('name') != BALL_MODEL or [lk.get('name') for lk in links] != [BALL_LINK]:
        raise SystemExit(f'{BALL_SDF}: not model {BALL_MODEL} with the one link {BALL_LINK}; '
                         'the pickup weld names them')
    poses = [p.text.split() for p in model.iter('pose') if p.text]
    col = links[0].find('collision')
    if any(float(v) for p in poses for v in p) or col.find('pose') is not None:
        raise SystemExit(f'{BALL_SDF}: offset poses; the resting height assumes a centred sphere')
    bits = col.findtext('surface/contact/collide_bitmask') or '0xffff'
    b = int(bits, 16)
    if not all(b & int(m, 16) for m in (NET_BITMASK, RING_BITMASK, PICKUP_FLOOR_BITMASK)):
        raise SystemExit(f'{BALL_SDF}: bitmask {bits} misses the floor, the ring or the net')
    if b & int(TIP_BITMASK, 16):
        raise SystemExit(f"{BALL_SDF}: bitmask {bits} hits QUAD4's magnet ({TIP_BITMASK})")
    return float(col.findtext('geometry/sphere/radius')), bits


def ball_clearance(base, start='free'):
    """(nearest carrier, newcomer) from the ball centre on the floor: carrier centres, and QUAD4's
    body-to-tip segment (laid out along +y, or welded: from its body in to plate 9)."""
    carriers = re.findall(r'<name>x3_drone[0-2]</name>\s*<pose>(\S+) (\S+)', base)
    if len(carriers) != 3:
        raise SystemExit('carrier poses not found in the base world')
    near_carrier = min(math.dist(BALL_XY, (float(x), float(y))) for x, y in carriers)
    (x, y, _), roll = spawn(start)
    tip_y = y + CABLE_LEN * math.sin(roll)        # free: out along +y; welded: in to the plate
    lo, hi = min(y, tip_y), max(y, tip_y)
    return near_carrier, math.dist(BALL_XY, (x, min(max(BALL_XY[1], lo), hi)))


def pickup_model(text, start='free'):
    """The newcomer (free or welded) + the partner model's pickup weld."""
    end = ('with the magnet on the floor. -->' if start == 'free' else 'tip on the plate. -->')
    text = _replace_once(text, end, (
        end[:-len(' -->')] + "\n         With the pickup weld onto Tejen's ball "
        f'(/pickup/*) for {PICKUP_WORLDS[start]}. -->'), f'{start} header')
    close = '    </model>\n</sdf>'
    return _replace_once(text, close, PICKUP_WELD + close, 'closing </model>')


def net(scale, top_z, bitmask):
    """The partner world's net scaled about the ring axis and its top face (top_z): panels
    sagging from the rim to a flat centre patch. Collision and visual only, no mass."""
    bits = f'<surface><contact><collide_bitmask>{bitmask}</collide_bitmask></contact></surface>'
    r, z = NET_PANEL_R * scale, top_z + NET_PANEL_DZ * scale
    out = []
    for k in range(NET_PANELS):
        th = 2.0 * math.pi * k / NET_PANELS
        pose = (f'<pose>{r * math.cos(th):.4f} {r * math.sin(th):.4f} {z:.4f} '
                f'0 {-NET_PITCH:.4f} {th:.4f}</pose>')
        size = f'{NET_PANEL_LEN * scale:.4f} {NET_PANEL_W * scale:.4f}'
        out.append(f'          <collision name="net_{k}">{pose}<geometry><box><size>{size} '
                   f'{NET_THICK:.3f}</size></box></geometry>{bits}</collision>\n'
                   f'          <visual name="net_{k}_visual">{pose}<geometry><box><size>{size} '
                   f'{NET_VIS_THICK:.3f}</size></box></geometry>{NET_LOOK}</visual>\n')
    pose = f'<pose>0 0 {top_z + NET_CENTRE_DZ * scale:.4f} 0 0 0</pose>'
    disc = f'<cylinder><radius>{NET_CENTRE_R * scale:.4f}</radius><length>'
    out.append(f'          <collision name="net_centre">{pose}<geometry>{disc}{NET_THICK:.3f}'
               f'</length></cylinder></geometry>{bits}</collision>\n'
               f'          <visual name="net_centre_visual">{pose}<geometry>{disc}'
               f'{NET_VIS_THICK:.3f}</length></cylinder></geometry>{NET_LOOK}</visual>\n')
    return ''.join(out)


def open_ring():
    """The annulus that replaces the solid disc, and the net across its inner circle."""
    rc, width = (RING_INNER_R + RING_OUTER_R) / 2.0, RING_OUTER_R - RING_INNER_R
    chord = 2.0 * RING_OUTER_R * math.tan(math.pi / ANNULUS_SEGMENTS)   # closes the outer edge
    scale = RING_INNER_R / LEGACY_NET_RIM_R
    bits = f'<surface><contact><collide_bitmask>{RING_BITMASK}</collide_bitmask></contact></surface>'
    out = [f"""          <!-- Combined M1 subtest: the ring is open with a net across the inner circle, as the rig
               ring (the ball rolled off the partner world's solid disc, R0625). Annulus r {RING_INNER_R:.3f}-{RING_OUTER_R:.3f}:
               the {1000 * width:.0f} mm rim centred on the plates (r {ATTACH_RADIUS}), as ring_k_visual, in {ANNULUS_SEGMENTS} boxes
               of the disc's thickness and height, {RING_BITMASK} as the disc (the floor and the drones, not
               the newcomer's magnet {TIP_BITMASK}). The net is the partner world's, scaled by {scale:.3f} (its rim
               r {LEGACY_NET_RIM_R:.2f}) about the axis and the top face; collision and visual only, no mass. Bit
               {NET_BITMASK}: the ball lands in it, the magnet ({TIP_BITMASK}) and the floor ({PICKUP_FLOOR_BITMASK}) pass through it. -->
"""]
    for k in range(ANNULUS_SEGMENTS):
        th = 2.0 * math.pi * k / ANNULUS_SEGMENTS
        out.append(f'          <collision name="ring_annulus_{k}"><pose>{rc * math.cos(th):.4f} '
                   f'{rc * math.sin(th):.4f} 0 0 0 {th:.4f}</pose><geometry><box><size>'
                   f'{width:.3f} {chord:.4f} {RING_THICKNESS:.3f}</size></box></geometry>{bits}'
                   '</collision>\n')
    return ''.join(out) + net(scale, RING_THICKNESS / 2.0, NET_BITMASK)


def _pickup_header(start):
    if start == 'free':
        return ('<!-- GENERATED by tools/make_attach_worlds.py: do not edit. Rig-twin combined M1 subtest '
                '(Wesley 10 Oct): three carriers on the floor at '
                f'{CARRIER_AZIMUTHS.replace(",", "/")} deg (rig plates 3/11/7) lift the ring; Tejen\'s '
                'controller takes QUAD4 (x3_drone3, free on the floor at (0, 1.4)) off, picks up his ball '
                f'(on the floor at ({BALL_XY[0]}, {BALL_XY[1]})), drops it into the ring\'s net, then '
                'attaches on rig plate 9 (90 deg). The free-start attach world with the pickup newcomer, an '
                f'open ring (annulus r {RING_INNER_R:.3f}-{RING_OUTER_R:.3f} + net, bit {NET_BITMASK}) '
                f'and the floor at {PICKUP_FLOOR_BITMASK}. -->\n')
    return ('<!-- GENERATED by tools/make_attach_worlds.py: do not edit. Rig-twin full M1 (Wesley 10 Oct): '
            f'three carriers on the floor at {CARRIER_AZIMUTHS.replace(",", "/")} deg (rig plates 3/11/7) and '
            'QUAD4 (x3_drone3) welded on rig plate 9 (90 deg) lift the ring; QUAD4 detaches, steps clear and '
            f'is released to Tejen\'s controller, which picks up his ball (on the floor at ({BALL_XY[0]}, '
            f'{BALL_XY[1]})), drops it into the ring\'s net, then attaches on plate 9 again. The welded-start '
            'attach world with the welded pickup newcomer, an open ring (annulus r '
            f'{RING_INNER_R:.3f}-{RING_OUTER_R:.3f} + net, bit {NET_BITMASK}) and the floor at '
            f'{PICKUP_FLOOR_BITMASK}. -->\n')


def pickup_world(world, base, ball_text, start='free'):
    """The attach world (free start: the combined M1 subtest; welded: the full M1) with the pickup
    newcomer, the open ring with its net, the floor under the net masked off, and Tejen's ball (the
    copy) on the floor."""
    seg = re.search(r'<visual name="ring_0_visual"><pose>(\S+) .*?<size>\S+ (\S+) ', base)
    if (abs(float(seg[1]) - float(seg[2]) / 2 - RING_INNER_R) > 1e-6
            or abs(float(seg[1]) + float(seg[2]) / 2 - RING_OUTER_R) > 1e-6):
        raise SystemExit('the annulus does not match the ring visuals')
    ball_z, ball_bits = ball(ball_text)
    near_carrier, near_newcomer = ball_clearance(base, start)
    if near_carrier < BALL_CLEAR_CARRIER or near_newcomer < BALL_CLEAR_NEWCOMER:
        raise SystemExit(f'ball at {BALL_XY}: {near_carrier:.2f} m from a carrier, '
                         f'{near_newcomer:.2f} m from the newcomer')
    text = _replace_once(world, _header(start), _pickup_header(start), f'{start} header')
    text = _replace_once(text, FLOOR_COLLISION, FLOOR_COLLISION.replace(
        '</geometry>', f'</geometry><surface><contact><collide_bitmask>{PICKUP_FLOOR_BITMASK}'
        '</collide_bitmask></contact></surface>', 1) + (
        f'<!-- bits {RING_BITMASK} the ring, {TIP_BITMASK} the magnet, and every unmasked '
        f'model: the net ({NET_BITMASK}) passes through the floor -->'), 'floor collision')
    disc = ('          <collision name="collision"><pose>0 0 0 0 0 0</pose>\n            '
            + RING_COLLISION_MASKED + '\n')
    text = _replace_once(text, disc, open_ring(), 'ring disc')
    text = _replace_once(text, f'<uri>models/{MODELS[start]}</uri>',
                         f'<uri>models/{PICKUP_MODELS[start]}</uri>', 'newcomer uri')
    note = '(detach_on_start).' if start == 'free' else '(attached_at_start).'
    text = _replace_once(text, f'{note} -->', (
        f"{note}\n         The pickup weld onto Tejen's ball forms at spawn too; "
        'partner_mission.launch.py frees it\n         (/pickup/detach) before his mission. -->'),
        'newcomer note')
    (x, y), r = BALL_XY, ball_z
    beside = "QUAD4's laid-out arm" if start == 'free' else "QUAD4's body and rod (welded)"
    block = f"""
    <!-- ===== TEJEN'S BALL: his pickup object ({BALL_SDF} as models/{BALL_COPY}, r {r:g}, bitmask {ball_bits}) =====
         On the floor at ({x}, {y}), resting on its radius: {near_newcomer:.2f} m beside {beside},
         {near_carrier:.2f} m from the nearest carrier. His object magnet welds it to QUAD4's tip (/pickup/attach) and
         drops it into the ring's net (/pickup/detach). /model/{BALL_MODEL}/pose: [link, model]. -->
    <include>
      <uri>models/{BALL_COPY}</uri>
      <pose>{_f(x)} {_f(y)} {_f(r)} 0 0 0</pose>
{BALL_POSE_PUBLISHER}    </include>
"""
    return _replace_once(text, '  </world>\n</sdf>', block + '  </world>\n</sdf>', 'world end')


def generate(out_dir):
    """Write the newcomer models and the worlds under out_dir (same layout as ASSETS)."""
    os.makedirs(os.path.join(out_dir, 'models'), exist_ok=True)
    files = {os.path.join('models', name): newcomer_model(start)
             for start, name in MODELS.items()}
    base = carrier_world()
    files.update({name: attach_world(base, start) for name, start in WORLDS.items()})
    files[os.path.join('models', BALL_COPY)] = ball_model()
    for start in ('free', 'welded'):
        files[os.path.join('models', PICKUP_MODELS[start])] = pickup_model(
            files[os.path.join('models', MODELS[start])], start)
        files[PICKUP_WORLDS[start]] = pickup_world(
            files[next(n for n, s in WORLDS.items() if s == start)], base,
            files[os.path.join('models', BALL_COPY)], start)
    files.update({name: _replace_once(files[src], USER_COMMANDS, USER_COMMANDS + WRENCH_PLUGIN,
                                      'user commands plugin')
                  for name, src in PUSH_WORLDS.items()})
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
