"""Rig-twin attach newcomer and worlds (tools/make_attach_worlds.py): the generated files are
current, the newcomer is QUAD4 on the twin's rod, and the worlds put the carriers on the M1
layout with the newcomer free on the floor or welded on plate 9.

    python3 -m pytest tools/test/test_make_attach_worlds.py -q
"""
import math
import pathlib
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
LINKS = ['base_link', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3', 'magnet_arm', 'magnet_tip_link']


def _model(start):
    return ET.parse(ASSETS / 'models' / MODELS[start]).getroot().find('model')


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


@pytest.mark.parametrize('world', [FREE, WELDED])
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
@pytest.mark.parametrize('rel', [FREE, WELDED] + [f'models/{m}' for m in MODELS.values()])
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
