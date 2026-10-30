"""Rig-twin carry worlds (tools/make_carry_worlds.py) and the generator options behind
them. The legacy worlds (simulation_assets/old_worlds/ since 5 Oct 2026) must stay
byte-identical: the attach, M1, M2 and weld work is pinned to them (decisions.md, 30 Sep 2026).

    python3 -m pytest tools/test/test_rig_worlds.py -q
"""
import math
import pathlib
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

REPO = pathlib.Path(__file__).resolve().parents[2]
ASSETS = REPO / 'simulation_assets'
OLD = ASSETS / 'old_worlds'
sys.path.insert(0, str(REPO / 'tools'))

import make_carry_worlds as mcw                                   # noqa: E402
from check_geometry import world_geometry, config_mismatches      # noqa: E402

LEGACY = {
    'four_rigid_ground.sdf': ['--n', '4', '--ground-start'],
    'three_rigid_ground.sdf': ['--n', '3', '--ground-start'],
    'two_rigid_ground.sdf': ['--n', '2', '--ground-start'],
    'four_rigid_ground_3915.sdf': ['--n', '4', '--ground-start', '--azimuths', '30,90,150,270'],
    'three_rigid_ground_m10.sdf': ['--n', '3', '--ground-start', '--payload-mass', '1.0'],
    'four_rigid_2026-09.sdf': ['--n', '4', '--payload-z', '0.6'],
}
RIG = sorted(mcw.WORLDS)


@pytest.mark.parametrize('name', sorted(LEGACY))
def test_generator_defaults_reproduce_the_legacy_worlds(name, tmp_path):
    out = tmp_path / name
    subprocess.run([sys.executable, str(ASSETS / 'generate_rigid_world.py'),
                    '--cable-len', '0.5', '--detachable', *LEGACY[name], '--out', str(out)],
                   check=True, cwd=ASSETS, stdout=subprocess.DEVNULL)
    assert out.read_bytes() == (OLD / name).read_bytes()


def test_rig_worlds_and_models_are_up_to_date():
    r = subprocess.run([sys.executable, str(REPO / 'tools/make_carry_worlds.py'), '--check'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout


@pytest.mark.parametrize('name', RIG)
def test_rig_world_reads_back_the_rig_geometry(name):
    g = world_geometry(ASSETS / name)
    for k, v in mcw.world_expect(name).items():
        assert abs(g[k] - v) < 1e-4, f'{name}: {k} {g[k]:.4f} != {v}'
    assert not [k for k in g if k.startswith('_warn')], g
    ixx, izz = mcw.expected_inertia(name)
    assert g['load_ixx'] == pytest.approx(ixx, rel=1e-3) and g['load_izz'] == pytest.approx(izz, rel=1e-3)


def test_the_rig_ring_inertia_is_the_annulus_per_kg():
    # outer 0.255, inner 0.195, 40 mm; the S ring R 0.5 m: outer 0.53, inner 0.47, 0.9 kg (card 2026-10-23_scalability)
    for name, mass in (('four_rigid_ground_rig.sdf', 0.86), ('four_rigid_ground_rig_m070.sdf', 0.70)):
        assert mcw.expected_inertia(name) == pytest.approx((2.227e-2 * mass / 0.86, 4.431e-2 * mass / 0.86), rel=1e-3)
    assert mcw.expected_inertia('eight_rigid_ground_rig_R50_L100.sdf') == pytest.approx((0.113025, 0.22581))


def test_rig_layouts():
    az = {n: world_geometry(ASSETS / n)['attach_azimuths'] for n in RIG}
    assert az['four_rigid_ground_rig.sdf'] == [0.0, 90.0, 180.0, 270.0]
    assert az['four_rigid_ground_rig_3915.sdf'] == [30.0, 90.0, 150.0, 270.0]
    # the M1 layout at 1 m (Wesley 22 Oct): plates 3/11/7 and QUAD4's 9 as a fourth carrier
    assert az['four_rigid_ground_rig_3915_L100.sdf'] == [30.0, 90.0, 150.0, 270.0]
    assert world_geometry(ASSETS / 'four_rigid_ground_rig_3915_L100.sdf')['cable_len'] == 1.0
    assert az['three_rigid_ground_rig.sdf'] == [0.0, 120.0, 240.0]
    assert world_geometry(ASSETS / 'three_rigid_ground_rig.sdf')['num_drones'] == 3
    assert az['eight_rigid_ground_rig_R50_L100.sdf'] == [45.0 * k for k in range(8)]
    assert az['six_rigid_ground_rig_R50_L100.sdf'] == [60.0 * k for k in range(6)]
    assert az['nine_rigid_ground_rig_R53_L100.sdf'] == [40.0 * k for k in range(9)]


def test_the_1m_tier_b_and_push_worlds():
    """Thesis R (card 2026-10-23_refly_1m, Wesley 23 Oct): each is its 0.55 m twin's layout on 1 m rods; a _push
    world is its 1 m world plus the runner's WRENCH tool."""
    for name in ('four_rigid_ground_rig_3969', 'four_rigid_ground_rig_gap135', 'four_rigid_ground_rig_gap165',
                 'four_rigid_ground_rig_gap180', 'three_rigid_ground_rig_push'):
        g, twin = world_geometry(ASSETS / f'{name}_L100.sdf'), world_geometry(ASSETS / f'{name}.sdf')
        assert g['cable_len'] == pytest.approx(1.0) and twin['cable_len'] == pytest.approx(0.55)
        assert g['attach_azimuths'] == twin['attach_azimuths']
        for k in ('attach_radius', 'attach_z', 'load_mass', 'drone_mass', 'pivot_dz', 'num_drones'):
            assert g[k] == pytest.approx(twin[k], abs=1e-5), (name, k)
    # ... and the rejoin leaning-pair test's T (card 2026-10-23_rejoin_departed, Wesley 23 Oct ~08:00)
    for name in ('three_rigid_ground_rig_push_L100.sdf', 'three_rigid_ground_rig_T_push_L100.sdf'):
        push = (ASSETS / name).read_text().splitlines()
        base = (ASSETS / name.replace('_push', '')).read_text().splitlines()
        added = [ln for ln in push if ln not in base]
        assert len(push) == len(base) + 2 and [ln for ln in push if ln not in added] == base, name
        assert 'gz::sim::systems::ApplyLinkWrench' in added[1]
        assert 'ApplyLinkWrench' not in '\n'.join(base)
    assert world_geometry(ASSETS / 'three_rigid_ground_rig_T_push_L100.sdf')['attach_azimuths'] == [0.0, 180.0, 270.0]


def test_the_hub_world_is_the_1m_four_with_every_rod_at_the_centre():
    """The paper's own geometry (card 2026-10-23_paper_method_compare, Wesley 23 Oct): four_rigid_ground_rig_L100 with
    every rod hinged on a 1 mm hub at the ring's centre on the plates' plane; the ring (plates still drawn on the rim),
    rods, quads, masses and the rods' pose publishers as before; the hub only adds visuals."""
    name, base = 'four_rigid_ground_rig_hub_L100.sdf', 'four_rigid_ground_rig_L100.sdf'
    g, b = world_geometry(ASSETS / name), world_geometry(ASSETS / base)
    assert g['attach_radius'] == pytest.approx(0.001, abs=1e-6) and b['attach_radius'] == pytest.approx(0.225)
    assert g['attach_azimuths'] == b['attach_azimuths'] == [0.0, 90.0, 180.0, 270.0]
    for k in ('attach_z', 'cable_len', 'load_mass', 'load_ixx', 'load_izz', 'drone_mass', 'rod_mass', 'pivot_dz',
              'num_drones'):
        assert g[k] == pytest.approx(b[k], abs=1e-6), k
    lift = _lift(name)
    for i in range(4):
        stub = _xyz(lift.find(f"link[@name='stub_pay_{i}']"))
        assert math.hypot(stub[0], stub[1]) == pytest.approx(0.001, abs=1e-6) and stub[2] == pytest.approx(0.04)
        d = _xyz(next(inc for inc in lift.findall('include') if inc.findtext('name') == f'x3_drone{i}'))
        assert math.dist(stub, [d[0], d[1], d[2] - 0.04]) == pytest.approx(1.0, abs=1e-5)
    body, base_body = (_lift(w).find("model[@name='payload']/link[@name='body']") for w in (name, base))
    plates = [v for v in body.findall('visual') if v.get('name').startswith('plate_')]
    assert len(plates) == 12 and all(math.hypot(*_xyz(v)[:2]) == pytest.approx(0.225, abs=1e-5) for v in plates)
    added = {v.get('name') for v in body.findall('visual')} - {v.get('name') for v in base_body.findall('visual')}
    assert added == {'hub_visual'} | {f'spoke_{k}_visual' for k in range(4)}
    for tag in ('inertial', 'collision'):
        assert ET.tostring(body.find(tag)) == ET.tostring(base_body.find(tag))
    text, base_text = ((ASSETS / w).read_text() for w in (name, base))
    assert text.count('gz-sim-pose-publisher-system') == base_text.count('gz-sim-pose-publisher-system') == 5


def test_world_c_hangs_the_hub_at_the_centre_of_mass_and_the_rods_from_the_drone_centres():
    """Quan et al. exactly (card 2026-10-23_quan_exact, Wesley 23 Oct ~21:45): the hub world with the 1 mm hub on the
    ring's centre of mass and each rod's other end at its drone's centre, the paper's model to the letter; the ring,
    rods, quads, masses and pose publishers as the hub world's."""
    name, base = 'four_rigid_ground_rig_hubcom_L100.sdf', 'four_rigid_ground_rig_hub_L100.sdf'
    g, b = world_geometry(ASSETS / name), world_geometry(ASSETS / base)
    assert g['attach_radius'] == pytest.approx(0.001, abs=1e-6)
    assert g['attach_z'] == pytest.approx(0.0, abs=1e-6) and b['attach_z'] == pytest.approx(0.02)
    assert g['pivot_dz'] == pytest.approx(0.0, abs=1e-6) and b['pivot_dz'] == pytest.approx(0.04)
    assert g['attach_azimuths'] == b['attach_azimuths'] == [0.0, 90.0, 180.0, 270.0]
    for k in ('cable_len', 'load_mass', 'load_ixx', 'load_izz', 'drone_mass', 'rod_mass', 'num_drones'):
        assert g[k] == pytest.approx(b[k], abs=1e-6), k
    lift = _lift(name)
    ring_z = _xyz(lift.find("model[@name='payload']"))[2]
    for i in range(4):
        stub = _xyz(lift.find(f"link[@name='stub_pay_{i}']"))
        assert math.hypot(stub[0], stub[1]) == pytest.approx(0.001, abs=1e-6) and stub[2] == pytest.approx(ring_z)
        assert lift.find(f"joint[@name='tether_{i}_to_drone{i}']").find('pose') is None     # at the drone centre
        d = _xyz(next(inc for inc in lift.findall('include') if inc.findtext('name') == f'x3_drone{i}'))
        assert math.dist(stub, d) == pytest.approx(1.0, abs=1e-5)
    body, base_body = (_lift(w).find("model[@name='payload']/link[@name='body']") for w in (name, base))
    assert {v.get('name') for v in body.findall('visual')} == {v.get('name') for v in base_body.findall('visual')}
    for tag in ('inertial', 'collision'):
        assert ET.tostring(body.find(tag)) == ET.tostring(base_body.find(tag))
    assert (ASSETS / name).read_text().count('gz-sim-pose-publisher-system') == 5


def test_the_s_rings_have_a_plate_per_quad_and_a_real_time_cap():
    for name, n in (('eight_rigid_ground_rig_R50_L100.sdf', 8), ('six_rigid_ground_rig_R50_L100.sdf', 6)):
        root = ET.parse(ASSETS / name).getroot()
        assert root.findtext('world/physics/real_time_factor') == '0.4'
        body = _lift(name).find("model[@name='payload']/link[@name='body']")
        plates = [v for v in body.findall('visual') if v.get('name').startswith('plate_')]
        assert len(plates) == n
        for k, v in enumerate(plates):                         # one under each quad's magnet (poses to 1 um)
            x, y = _xyz(v)[:2]
            assert math.degrees(math.atan2(y, x)) % 360.0 == pytest.approx(360.0 * k / n, abs=1e-3)
            assert math.hypot(x, y) == pytest.approx(0.5, abs=1e-5)
    assert ET.parse(ASSETS / 'four_rigid_ground_rig.sdf').getroot().findtext(
        'world/physics/real_time_factor') == '1.0'


def _lift(name):
    return next(m for m in ET.parse(ASSETS / name).getroot().find('world').findall('model')
                if m.get('name') == 'lift_system')


def _xyz(elem):
    return [float(v) for v in elem.find('pose').text.split()[:3]]


def test_pivot_is_below_the_drone_centre_and_the_rod_is_0p55():
    lift = _lift('four_rigid_ground_rig.sdf')
    for i in range(4):
        j = lift.find(f"joint[@name='tether_{i}_to_drone{i}']")
        assert _xyz(j) == pytest.approx([0.0, 0.0, -0.04])       # in base_link
        drone = next(inc for inc in lift.findall('include')
                     if inc.findtext('name') == f'x3_drone{i}')
        assert drone.findtext('uri') == f'models/x3_rig_drone{i}.sdf'
        stub = _xyz(lift.find(f"link[@name='stub_pay_{i}']"))
        d = _xyz(drone)
        pivot = [d[0], d[1], d[2] - 0.04]
        assert math.dist(stub, pivot) == pytest.approx(0.55, abs=1e-5)
        assert math.hypot(stub[0], stub[1]) == pytest.approx(0.225, abs=1e-6)


def test_floor_start_rests_on_the_floor():
    lift = _lift('four_rigid_ground_rig.sdf')
    payload = lift.find("model[@name='payload']")
    assert _xyz(payload)[2] == pytest.approx(0.020)               # 40 mm ring on the floor
    for inc in lift.findall('include'):
        assert _xyz(inc)[2] == pytest.approx(0.10)                # same rest as the legacy
    assert ET.parse(ASSETS / 'four_rigid_ground_rig.sdf').getroot().find(
        "world/model[@name='platform_0']") is None


def test_rod_and_magnet_masses():
    rod = _lift('four_rigid_ground_rig.sdf').find("model[@name='tether_0']/link[@name='rod']")
    assert float(rod.findtext('inertial/mass')) == pytest.approx(0.075)
    # magnet at the payload end pulls the CoM 0.035*0.275/0.075 below the rod centre
    assert _xyz(rod.find('inertial'))[2] == pytest.approx(-0.035 * 0.275 / 0.075, abs=1e-6)


def test_rig_models_only_change_the_base_mass_and_inertia():
    for i in range(4):
        legacy = (ASSETS / 'models' / f'x3_drone{i}.sdf').read_text().splitlines()
        rig = (ASSETS / 'models' / f'x3_rig_drone{i}.sdf').read_text().splitlines()
        rig = [ln for ln in rig if 'Rig twin' not in ln and '(base 0.435' not in ln]
        diff = [(a.strip(), b.strip()) for a, b in zip(legacy, rig) if a != b]
        assert len(legacy) == len(rig)
        assert [a[:5] for a, _ in diff] == ['<mass', '<ixx>', '<iyy>', '<izz>']


def test_models_above_four_are_drone0_renamed():
    """x3_rig_drone4.. (card 2026-10-23_scalability) are x3_rig_drone0 with its name, IMU and namespaces renamed."""
    base = (ASSETS / 'models' / 'x3_rig_drone0.sdf').read_text().splitlines()
    assert mcw.N_MODELS >= 8
    for i in range(4, mcw.N_MODELS):
        model = [ln for ln in (ASSETS / 'models' / f'x3_rig_drone{i}.sdf').read_text().splitlines()
                 if 'Rig twin of' not in ln]
        body = '\n'.join(model)
        assert 'x3_drone0' not in body and body.count(f'x3_drone{i}') == 7
        ref = [ln.replace('x3_drone0', f'x3_drone{i}') for ln in base if 'Rig twin of' not in ln]
        assert model == ref


def test_a_legacy_config_on_a_rig_world_is_refused():
    bad = {k for k, _, _ in config_mismatches(
        ASSETS / 'four_rigid_ground_rig.sdf', None,
        {'cable_len': 0.5, 'attach_radius': 0.25, 'drone_mass': 0.64, 'pivot_offset_z': 0.0})}
    assert {'cable_len', 'attach_radius', 'drone_mass', 'pivot_offset_z'} <= bad
    ok = config_mismatches(
        ASSETS / 'four_rigid_ground_rig.sdf', None,
        {'cable_len': 0.55, 'attach_radius': 0.225, 'attach_z': 0.02, 'drone_mass': 0.55,
         'pivot_offset_z': -0.04, 'load_mass': 0.86})
    assert {k for k, _, _ in ok} <= {'load_ixx', 'load_izz'}      # params.py literals


def test_legacy_worlds_read_a_centre_pivot():
    for name in ('four_rigid_ground.sdf', 'three_rigid_ground.sdf', 'four_rigid_2026-09.sdf'):
        g = world_geometry(OLD / name)
        assert g['pivot_dz'] == pytest.approx(0.0, abs=1e-6)
        assert g['drone_airframe_mass'] == pytest.approx(0.64)
