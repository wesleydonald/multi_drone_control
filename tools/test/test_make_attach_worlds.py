"""Rig-twin attach newcomer and worlds (tools/make_attach_worlds.py): the generated files are
current, the newcomer is QUAD4 on the twin's rod, and the worlds put the carriers on the M1
layout with the newcomer free on the floor or welded on plate 9. The combined M1 subtest world
(pickup) is the free world with the pickup newcomer, an open ring with a net, and Tejen's ball (a
copy on collide bits 0x05, clear of QUAD4's magnet); its push variant adds the WRENCH tool.

    python3 -m pytest tools/test/test_make_attach_worlds.py -q
"""
import hashlib
import math
import pathlib
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
ASSETS = REPO / 'simulation_assets'
sys.path.insert(0, str(REPO / 'tools'))

import make_attach_worlds as maw                                  # noqa: E402
import make_carry_worlds as mcw                                   # noqa: E402
from check_geometry import world_geometry                         # noqa: E402

FREE, WELDED = 'three_rigid_ground_rig_attach.sdf', 'three_rigid_ground_rig_attach_welded.sdf'
MODELS = {'free': 'x3_rig_drone3_magnet.sdf', 'welded': 'x3_rig_drone3_magnet_welded.sdf'}
PICKUP, PICKUP_MODEL = 'three_rigid_ground_rig_attach_pickup.sdf', 'x3_rig_drone3_magnet_pickup.sdf'
PICKUP_PUSH, BALL_COPY = 'three_rigid_ground_rig_attach_pickup_push.sdf', 'models/rig_pickup_ball.sdf'
LINKS = ['base_link', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3', 'magnet_arm', 'magnet_tip_link']


def _model(start):
    name = PICKUP_MODEL if start == 'pickup' else MODELS[start]
    return ET.parse(ASSETS / 'models' / name).getroot().find('model')


def _pose(elem):
    p = elem.find('pose')
    return [float(v) for v in p.text.split()] if p is not None else [0.0] * 6


def _newcomer(world):
    """The world's top-level x3_drone3 include (uri, pose)."""
    w = ET.parse(ASSETS / world).getroot().find('world')
    inc = [i for i in w.findall('include') if i.findtext('name') == 'x3_drone3']
    assert len(inc) == 1
    return inc[0].findtext('uri'), _pose(inc[0])


def _tip_world(world, start):
    """Forward kinematics of the spawn pose: body -> pivot (arm origin) -> tip."""
    _, (x, y, z, *_) = _newcomer(world)
    m = _model(start)
    arm = m.find("link[@name='magnet_arm']")
    ax, ay, az, roll, pitch, yaw = _pose(arm)
    assert (ax, ay, pitch, yaw) == (0.0, 0.0, 0.0, 0.0)
    drop = -_pose(m.find("link[@name='magnet_tip_link']"))[2]
    pivot = (x, y, z + az)
    return pivot, (pivot[0], pivot[1] + drop * math.sin(roll), pivot[2] - drop * math.cos(roll))


def test_generated_files_are_up_to_date():
    r = subprocess.run([sys.executable, str(REPO / 'tools/make_attach_worlds.py'), '--check'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr


@pytest.mark.parametrize('start', sorted(MODELS))
def test_newcomer_links_and_tip_last(start):
    m = _model(start)
    assert m.get('name') == 'x3_drone3'
    # PoseArray index 0 is the LAST declared link: the tip must stay last (magnet_index 0)
    assert [lk.get('name') for lk in m.findall('link')] == LINKS


@pytest.mark.parametrize('start', sorted(MODELS))
def test_newcomer_rod_masses_and_pivot(start):
    m = _model(start)
    links = {lk.get('name'): lk for lk in m.findall('link')}
    mass = {k: float(v.findtext('inertial/mass')) for k, v in links.items()}
    assert sum(mass.values()) == pytest.approx(0.55)                  # rig drone_mass
    assert sum(v for k, v in mass.items() if not k.startswith('magnet')) == pytest.approx(0.475)
    assert mass['magnet_arm'] == pytest.approx(0.040)
    assert mass['magnet_tip_link'] == pytest.approx(0.035)
    arm = links['magnet_arm']
    assert arm.find('pose').get('relative_to') == 'base_link'
    assert _pose(arm)[2] == pytest.approx(-0.04)                     # pivot 4 cm below centre
    rod = float(arm.findtext('visual/geometry/cylinder/length'))
    tip = links['magnet_tip_link']
    assert tip.find('pose').get('relative_to') == 'magnet_arm'
    assert _pose(tip)[:3] == pytest.approx([0.0, 0.0, -0.55])
    assert rod == pytest.approx(0.55)
    assert _pose(arm.find('inertial'))[2] == pytest.approx(-0.275)   # spread along the rod
    assert float(arm.findtext('inertial/inertia/ixx')) == pytest.approx(0.04 * 0.55 ** 2 / 12,
                                                                          rel=1e-3)
    assert 0.04 + rod == pytest.approx(0.59)                         # body to tip, hanging


@pytest.mark.parametrize('start', sorted(MODELS))
def test_newcomer_joints_are_damped_universals(start):
    m = _model(start)
    joints = {j.get('name'): j for j in m.findall('joint')}
    want = {'base_to_magnet_arm': ('base_link', 'magnet_arm', 0.002),
            'arm_to_magnet_tip': ('magnet_arm', 'magnet_tip_link', 1e-4)}
    for name, (parent, child, damping) in want.items():
        j = joints[name]
        assert j.get('type') == 'universal'
        assert (j.findtext('parent'), j.findtext('child')) == (parent, child)
        for ax in ('axis', 'axis2'):
            assert float(j.findtext(f'{ax}/dynamics/damping')) == pytest.approx(damping)
    assert _pose(joints['base_to_magnet_arm']) == [0.0] * 6          # at the arm origin


@pytest.mark.parametrize('start', sorted(MODELS))
def test_newcomer_plugins(start):
    m = _model(start)
    pubs = [p for p in m.findall('plugin') if p.get('name').endswith('PosePublisher')]
    assert len(pubs) == 1
    assert pubs[0].findtext('publish_model_pose') == 'true'
    assert pubs[0].findtext('publish_link_pose') == 'true'
    assert pubs[0].findtext('publish_nested_model_pose') == 'false'
    welds = [p for p in m.findall('plugin') if p.get('name').endswith('DetachableJoint')]
    assert len(welds) == 1                                           # no pickup weld
    w = welds[0]
    assert {k: w.findtext(k) for k in ('parent_link', 'child_model', 'child_link',
                                       'attach_topic', 'detach_topic', 'output_topic')} == {
        'parent_link': 'magnet_tip_link', 'child_model': 'lift_system::payload',
        'child_link': 'body', 'attach_topic': '/payload/attach',
        'detach_topic': '/payload/detach', 'output_topic': '/payload/detachable_joint_state'}


def test_newcomer_airframe_is_the_rig_drone():
    """Every line of x3_rig_drone3.sdf but its nested pose publisher appears, in order."""
    rig = (ASSETS / 'models' / 'x3_rig_drone3.sdf').read_text()
    rig = rig.replace(maw.NESTED_POSE_PUBLISHER, '').splitlines()
    for start in MODELS:
        lines = iter((ASSETS / 'models' / MODELS[start]).read_text().splitlines())
        missing = [ln for ln in rig if not any(ln == have for have in lines)]
        assert not missing, missing[:3]


def test_the_two_models_differ_only_in_the_arm_spawn_pose():
    a = (ASSETS / 'models' / MODELS['free']).read_text().splitlines()
    b = (ASSETS / 'models' / MODELS['welded']).read_text().splitlines()
    diff = [(x, y) for x, y in zip(a, b) if x != y]
    assert len(a) == len(b) and len(diff) == 2
    assert 'start:' in diff[0][0] and 'relative_to="base_link"' in diff[1][0]


def test_skid_rests_the_newcomer_at_the_carriers_height():
    base = _model('free').find("link[@name='base_link']")
    cols = {c.get('name'): c for c in base.findall('collision')}
    box_bottom = -float(cols['base_link_inertia_collision'].findtext(
        'geometry/box/size').split()[2]) / 2
    skid = cols['landing_skid_collision']
    skid_bottom = _pose(skid)[2] - float(skid.findtext('geometry/box/size').split()[2]) / 2
    assert skid_bottom == pytest.approx(box_bottom) == pytest.approx(-0.10)
    for world in (FREE, WELDED):
        assert _newcomer(world)[1][2] == pytest.approx(0.10)


@pytest.mark.parametrize('world', [FREE, WELDED, PICKUP, PICKUP_PUSH])
def test_carriers_are_the_twin_on_the_m1_layout(world):
    g = world_geometry(ASSETS / world)
    for k, v in mcw.EXPECT.items():
        assert g[k] == pytest.approx(v, abs=1e-4), k
    assert not [k for k in g if k.startswith('_warn')], g
    assert g['attach_azimuths'] == [30.0, 150.0, 270.0]
    lift = next(m for m in ET.parse(ASSETS / world).getroot().find('world').findall('model')
                if m.get('name') == 'lift_system')
    for i, az in enumerate((270.0, 30.0, 150.0)):                    # rig plates 3/11/7
        stub = _pose(lift.find(f"link[@name='stub_pay_{i}']"))
        assert math.degrees(math.atan2(stub[1], stub[0])) % 360 == pytest.approx(az, abs=1e-3)
        inc = next(c for c in lift.findall('include') if c.findtext('name') == f'x3_drone{i}')
        assert inc.findtext('uri') == f'models/x3_rig_drone{i}.sdf'
        assert _pose(inc)[2] == pytest.approx(0.10)                  # ground start
    assert _pose(lift.find("model[@name='payload']"))[:3] == pytest.approx([0.0, 0.0, 0.02])
    assert ET.parse(ASSETS / world).getroot().find("world/model[@name='platform_0']") is None


@pytest.mark.parametrize('world', [FREE, WELDED])
def test_worlds_add_only_the_newcomer_and_bitmasks_to_the_carrier_twin(world):
    base = maw.carrier_world().splitlines()
    have = (ASSETS / world).read_text().splitlines()
    added = [ln for ln in have if ln not in base]
    assert len(have) - len(base) == len(added)
    assert sum('collide_bitmask' in ln for ln in added) == 1
    assert sum('<include>' in ln for ln in added) == 1


def test_free_start_spawn():
    uri, pose = _newcomer(FREE)
    assert uri == 'models/x3_rig_drone3_magnet.sdf'
    assert pose == pytest.approx([0.0, 1.4, 0.10, 0.0, 0.0, 0.0])
    pivot, tip = _tip_world(FREE, 'free')
    assert pivot == pytest.approx((0.0, 1.4, 0.06))
    assert tip[2] == pytest.approx(maw.TIP_R, abs=1e-6)              # magnet resting on the floor
    assert tip[1] > 1.9                                              # laid outward (+y)
    assert math.dist(pivot, tip) == pytest.approx(0.55)


def test_welded_start_spawn_on_plate_9():
    uri, pose = _newcomer(WELDED)
    assert uri == 'models/x3_rig_drone3_magnet_welded.sdf'
    assert pose[:2] == pytest.approx([0.0, 0.775], abs=1e-3)
    assert pose[2:] == pytest.approx([0.10, 0.0, 0.0, 0.0])
    pivot, tip = _tip_world(WELDED, 'welded')
    assert tip == pytest.approx((0.0, 0.225, 0.04), abs=1e-5)        # plate 9 top, ring on floor
    # the rod at the carriers' ground-start elevation, the drone on their circle
    lift = next(m for m in ET.parse(ASSETS / WELDED).getroot().find('world').findall('model')
                if m.get('name') == 'lift_system')
    stub = _pose(lift.find("link[@name='stub_pay_0']"))
    d0 = _pose(next(c for c in lift.findall('include') if c.findtext('name') == 'x3_drone0'))
    assert pose[2] - 0.04 - tip[2] == pytest.approx(d0[2] - 0.04 - stub[2], abs=1e-6)
    assert math.hypot(*pose[:2]) == pytest.approx(math.hypot(*d0[:2]), abs=1e-6)


def test_magnet_never_touches_the_ring_but_rests_on_the_floor():
    tip = _model('free').find("link[@name='magnet_tip_link']/collision")
    tip_bits = int(tip.findtext('surface/contact/collide_bitmask'), 16)
    w = ET.parse(ASSETS / FREE).getroot().find('world')
    ring = next(m for m in w.iter('model') if m.get('name') == 'payload').find(
        "link[@name='body']/collision")
    ring_bits = int(ring.findtext('surface/contact/collide_bitmask'), 16)
    ground = w.find("model[@name='ground_plane']/link/collision")
    ground_bits = int(ground.findtext('surface/contact/collide_bitmask') or '0xffff', 16)
    assert tip_bits & ring_bits == 0
    assert tip_bits & ground_bits and ring_bits & ground_bits


@pytest.mark.skipif(shutil.which('gz') is None, reason='gz CLI not installed')
@pytest.mark.parametrize('rel', [FREE, WELDED, PICKUP, PICKUP_PUSH, BALL_COPY]
                         + [f'models/{m}' for m in [*MODELS.values(), PICKUP_MODEL]])
def test_gz_sdf_check(rel):
    r = subprocess.run(['gz', 'sdf', '-k', rel], cwd=ASSETS, capture_output=True, text=True)
    assert r.returncode == 0 and 'Valid' in r.stdout, r.stdout + r.stderr


def test_the_push_world_is_the_free_world_plus_the_wrench_tool():
    """Wesley 8 Oct night: a push variant for the newcomer fault test; the other worlds stay as they are."""
    free = (ASSETS / 'three_rigid_ground_rig_attach.sdf').read_text()
    push = (ASSETS / 'three_rigid_ground_rig_attach_push.sdf').read_text()
    assert push != free and push.replace(maw.WRENCH_PLUGIN, '', 1) == free
    assert 'ApplyLinkWrench' not in free
    ET.fromstring(push.split('\n', 1)[1].split('\n', 1)[1])          # well-formed below the two comment lines


# ===== the combined M1 subtest world (Wesley 10 Oct) =====

LEGACY_WORLD = ASSETS / 'old_worlds' / 'three_attach_partner_attached.sdf'
LEGACY_MODEL = ASSETS / 'models' / 'x3_drone3_magnet_rod_partner_attached.sdf'
BALL = ASSETS / 'tejen' / 'payload_model.sdf'
# the existing outputs when the pickup variant was added: it leaves them byte for byte. A deliberate
# change to one of them updates its digest here.
OLD_DIGESTS = {
    FREE: '807924f97b3872f23efe38c88d6473b0de9dab332cfbd487d1bfe1872714f076',
    WELDED: '587805ff1ce120845e0e61a7fceeb25e05aa881654993f10df8b2e3a528d830b',
    'three_rigid_ground_rig_attach_push.sdf':
        'aa2a7e30f41839aab8b4f4691f5acdfe52d02eec0e1a529c64f53990e7f80cd8',
    f"models/{MODELS['free']}": 'ff36ee07b5e489a5fe617b0d7dd65a68a992324f873430953828509716bf94f1',
    f"models/{MODELS['welded']}": '5839b65674a9cf31e03937a0a5940c2b13b219ef4bfaf33c75b410b306de3fd6',
}


def _world(name):
    return ET.parse(ASSETS / name).getroot().find('world')


def _bits(collision):
    return int(collision.findtext('surface/contact/collide_bitmask') or '0xffff', 16)


def _ring_body(world):
    return next(m for m in world.iter('model') if m.get('name') == 'payload').find(
        "link[@name='body']")


def _ball_include(world, uri=BALL_COPY):
    inc = [i for i in world.findall('include') if i.findtext('uri') == uri]
    assert len(inc) == 1
    return inc[0]


def _welds(model):
    keys = ('parent_link', 'child_model', 'child_link', 'attach_topic', 'detach_topic',
            'output_topic')
    plugins = [p for p in model.findall('plugin') if p.get('name').endswith('DetachableJoint')]
    return [{k: p.findtext(k) for k in keys} for p in plugins]


def _numbers(elem):
    """Every number under an element's geometry."""
    return [float(v) for v in ' '.join(elem.find('geometry').itertext()).split()]


def test_pickup_files_are_generated(tmp_path):
    rels = {pathlib.Path(r).as_posix() for r in maw.generate(tmp_path)}
    for rel in (PICKUP, f'models/{PICKUP_MODEL}', BALL_COPY, PICKUP_PUSH):
        assert rel in rels
        assert (tmp_path / rel).read_bytes() == (ASSETS / rel).read_bytes()


def test_pickup_variant_leaves_the_existing_outputs_byte_for_byte():
    for rel, digest in OLD_DIGESTS.items():
        assert hashlib.sha256((ASSETS / rel).read_bytes()).hexdigest() == digest, rel


def test_pickup_model_is_the_free_model_plus_the_pickup_weld():
    free = (ASSETS / 'models' / MODELS['free']).read_text().splitlines()
    pick = (ASSETS / 'models' / PICKUP_MODEL).read_text().splitlines()
    gone = [ln for ln in free if ln not in pick]
    assert len(gone) == 1 and gone[0].endswith('with the magnet on the floor. -->')   # header
    assert len(pick) - len(free) == 1 + len(maw.PICKUP_WELD.splitlines())
    # PoseArray index 0 is the LAST declared link: the tip must stay last (magnet_index 0)
    assert [lk.get('name') for lk in _model('pickup').findall('link')] == LINKS


def test_pickup_weld_names_the_ball_as_the_partner_model_does():
    ball = ET.parse(BALL).getroot().find('model')
    assert ball.get('name') == 'payload_model'
    assert [lk.get('name') for lk in ball.findall('link')] == ['payload_link']
    ring, pickup = _welds(_model('pickup'))
    assert ring == _welds(_model('free'))[0]                         # the ring weld as before
    assert pickup == {'parent_link': 'magnet_tip_link', 'child_model': ball.get('name'),
                      'child_link': ball.find('link').get('name'),
                      'attach_topic': '/pickup/attach', 'detach_topic': '/pickup/detach',
                      'output_topic': '/pickup/detachable_joint_state'}
    # its comments hold '--', which ElementTree rejects: parse the plugins alone
    legacy = re.findall(r'<plugin [^>]*DetachableJoint">.*?</plugin>', LEGACY_MODEL.read_text(), re.S)
    legacy = _welds(ET.fromstring('<model>' + ''.join(legacy) + '</model>'))
    assert pickup in legacy


def test_pickup_world_is_the_free_world_with_the_listed_changes():
    free = (ASSETS / FREE).read_text().splitlines()
    pick = (ASSETS / PICKUP).read_text().splitlines()
    gone = [ln.strip() for ln in free if ln not in pick]
    assert gone[0].startswith('<!-- GENERATED') and 'free start' in gone[0]
    assert gone[1].startswith('<collision name="collision"><geometry><plane>')      # the floor
    assert gone[2].startswith('<collision name="collision"><pose>0 0 0 0 0 0</pose>')
    assert gone[3].startswith('<geometry><cylinder><radius>0.255</radius>')        # the disc
    assert gone[4].startswith('<surface><contact><collide_bitmask>0x01')
    assert gone[5] == '</collision>'
    assert gone[6].endswith('(detach_on_start). -->')
    assert gone[7] == f"<uri>models/{MODELS['free']}</uri>"
    assert len(gone) == 8
    added = [ln for ln in pick if ln not in free]
    assert pick.count('    <include>') - free.count('    <include>') == 1   # the ball
    assert sum('collide_bitmask' in ln for ln in added) == 1 + 16 + 17  # floor, annulus, net
    assert _newcomer(PICKUP) == (f'models/{PICKUP_MODEL}', _newcomer(FREE)[1])


def test_pickup_ring_is_an_annulus_at_the_rings_real_radii():
    disc = _ring_body(_world(FREE)).find('collision')
    r_out = float(disc.findtext('geometry/cylinder/radius'))
    thick = float(disc.findtext('geometry/cylinder/length'))
    body = _ring_body(_world(PICKUP))
    assert body.find("collision[@name='collision']") is None          # no solid disc
    vis = body.find("visual[@name='ring_0_visual']")                  # the rim as drawn
    rc = math.hypot(*_pose(vis)[:2])
    width = float(vis.findtext('geometry/box/size').split()[1])
    r_in = rc - width / 2
    assert (r_in, rc + width / 2) == pytest.approx((0.195, r_out))
    assert rc == pytest.approx(mcw.ATTACH_RADIUS)                     # the plates centred on it
    boxes = [c for c in body.findall('collision') if c.get('name').startswith('ring_annulus_')]
    assert len(boxes) == maw.ANNULUS_SEGMENTS
    for c in boxes:
        x, y, z, roll, pitch, yaw = _pose(c)
        sx, sy, sz = (float(v) for v in c.findtext('geometry/box/size').split())
        assert math.hypot(x, y) == pytest.approx((r_in + r_out) / 2, abs=1e-4)
        assert abs(math.remainder(math.atan2(y, x) - yaw, 2 * math.pi)) < 1e-3
        assert (sx, sz, z, roll, pitch) == pytest.approx((r_out - r_in, thick, _pose(disc)[2], 0, 0),
                                                         abs=1e-4)
        assert sy >= 2 * r_out * math.tan(math.pi / len(boxes)) - 1e-4   # outer edge closed


def test_pickup_net_is_the_partner_net_scaled_to_the_inner_radius():
    legacy_body = _ring_body(ET.parse(LEGACY_WORLD).getroot().find('world'))
    a0 = legacy_body.find("collision[@name='ring_annulus_0']")
    a0_size = [float(v) for v in a0.findtext('geometry/box/size').split()]
    legacy_rim = _pose(a0)[0] - a0_size[0] / 2
    legacy_top = _pose(a0)[2] + a0_size[2] / 2
    assert maw.LEGACY_NET_RIM_R == pytest.approx(legacy_rim)
    legacy = [e for e in legacy_body if (e.get('name') or '').startswith('net_')]
    # the generator at scale 1 reproduces the partner world's net
    ours = list(ET.fromstring('<link>' + maw.net(1.0, legacy_top, '0x02') + '</link>'))
    assert [e.get('name') for e in ours] == [e.get('name') for e in legacy]
    for a, b in zip(ours, legacy):
        assert _pose(a) == pytest.approx(_pose(b), abs=1e-4)
        assert _numbers(a) == pytest.approx(_numbers(b), abs=1e-4)
        if a.tag == 'collision':
            assert _bits(a) == _bits(b) == 0x02
    # here: scaled about the axis and the top face to the inner radius, panel thickness kept
    s = maw.RING_INNER_R / legacy_rim
    top = mcw.RING_THICKNESS / 2
    body = _ring_body(_world(PICKUP))
    net = [e for e in body if (e.get('name') or '').startswith('net_')]
    assert [e.get('name') for e in net] == [e.get('name') for e in legacy]
    for a, b in zip(net, legacy):
        x, y, z, *rpy = _pose(b)
        assert _pose(a) == pytest.approx([s * x, s * y, top + s * (z - legacy_top), *rpy], abs=2e-4)
        want = _numbers(b)
        want = [s * v for v in want[:-1]] + want[-1:]                 # thickness unscaled
        assert _numbers(a) == pytest.approx(want, abs=2e-4)
    # the panels reach the annulus at the top face and overlap the centre patch
    patch = float(body.findtext("collision[@name='net_centre']/geometry/cylinder/radius"))
    for c in (e for e in net if e.tag == 'collision' and e.get('name') != 'net_centre'):
        x, y, z, _, pitch, _ = _pose(c)
        half = float(c.findtext('geometry/box/size').split()[0]) / 2
        r, dr, dz = math.hypot(x, y), half * math.cos(pitch), -half * math.sin(pitch)
        assert maw.RING_INNER_R <= r + dr <= maw.RING_INNER_R + 0.005
        assert top - 0.002 <= z + dz <= top
        assert r - dr <= patch


def test_pickup_bitmasks():
    w = _world(PICKUP)
    cols = _ring_body(w).findall('collision')
    annulus = [c for c in cols if c.get('name').startswith('ring_annulus_')]
    net = [c for c in cols if c.get('name').startswith('net_')]
    assert len(cols) == len(annulus) + len(net) and len(net) == 17
    assert {_bits(c) for c in annulus} == {0x01}
    assert {_bits(c) for c in net} == {0x04}
    tip = _bits(_model('pickup').find("link[@name='magnet_tip_link']/collision"))
    assert tip == 0x02
    floor = _bits(w.find("model[@name='ground_plane']/link/collision"))
    ball = _bits(ET.parse(ASSETS / BALL_COPY).getroot().find('model/link/collision'))
    assert ball == 0x05
    assert ball & 0x04                                               # the ball lands in the net
    assert not tip & 0x04 and not floor & 0x04                       # magnet and floor pass through
    assert tip & floor and 0x01 & floor and ball & floor             # all rest on the floor
    assert not tip & 0x01 and ball & 0x01                            # the magnet never touches the ring
    assert not tip & ball                                            # nor the ball, as the legacy tip


def test_ball_rests_on_the_floor_clear_of_the_carriers_and_quad4():
    w = _world(PICKUP)
    inc = _ball_include(w)
    ball = _pose(inc)[:3]
    radius = float(ET.parse(ASSETS / BALL_COPY).getroot().findtext(
        'model/link/collision/geometry/sphere/radius'))
    assert ball[2] == pytest.approx(radius)
    assert inc.find('name') is None             # model payload_model: the weld's child_model
    lift = next(m for m in w.findall('model') if m.get('name') == 'lift_system')
    carriers = [_pose(c)[:3] for c in lift.findall('include')
                if c.findtext('name') in ('x3_drone0', 'x3_drone1', 'x3_drone2')]
    assert len(carriers) == 3
    assert min(math.dist(ball, c) for c in carriers) >= maw.BALL_CLEAR_CARRIER >= 1.0
    body = _newcomer(PICKUP)[1][:3]
    pivot, tip = _tip_world(PICKUP, 'pickup')
    arm = [body] + [tuple(p + i / 100 * (q - p) for p, q in zip(pivot, tip)) for i in range(101)]
    assert min(math.dist(ball, p) for p in arm) >= 1.0


def test_ball_pose_publisher_is_the_partner_worlds():
    def block(world, uri=BALL_COPY):
        pubs = _ball_include(world, uri).findall('plugin')
        assert len(pubs) == 1
        return pubs[0].attrib, [(c.tag, c.text) for c in pubs[0]]
    legacy = ET.parse(LEGACY_WORLD).getroot().find('world')
    assert block(_world(PICKUP)) == block(legacy, 'tejen/payload_model.sdf')


def test_the_ball_copy_is_his_ball_plus_the_bitmask():
    """Only the header and the collision's bitmask differ; the names stay (the pickup weld and his
    launch's /model/payload_model/pose bridge use them). His file is untouched."""
    his = BALL.read_text().splitlines()
    ours = (ASSETS / BALL_COPY).read_text().splitlines()
    added = [ln for ln in ours if ln not in his]
    assert [ln for ln in his if ln not in ours] == []
    assert len(ours) - len(his) == len(added) == 3
    assert 'GENERATED' in added[0]
    assert added[2].strip() == '<surface><contact><collide_bitmask>0x05</collide_bitmask></contact></surface>'
    model = ET.parse(ASSETS / BALL_COPY).getroot().find('model')
    assert model.get('name') == 'payload_model'
    assert [lk.get('name') for lk in model.findall('link')] == ['payload_link']
    assert ET.parse(BALL).getroot().find('model/link/collision/surface') is None


def test_the_pickup_push_world_is_the_pickup_world_plus_the_wrench_tool():
    """The combined M1 card's _abort arm: a runner WRENCH push on the ring during his descent."""
    pick = (ASSETS / PICKUP).read_text()
    push = (ASSETS / PICKUP_PUSH).read_text()
    assert push != pick and push.replace(maw.WRENCH_PLUGIN, '', 1) == pick
    assert 'ApplyLinkWrench' not in pick and push.count('ApplyLinkWrench') == 1
    ET.fromstring(push.split('\n', 1)[1].split('\n', 1)[1])          # well-formed below the two comment lines
