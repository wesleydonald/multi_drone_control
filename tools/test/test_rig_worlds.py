"""Rig-twin carry worlds (tools/make_carry_worlds.py) and the generator options behind
them. The legacy worlds must stay byte-identical: the attach, M1, M2 and weld work is
pinned to them (decisions.md, 30 Sep 2026).

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
sys.path.insert(0, str(REPO / 'tools'))

import make_carry_worlds as mcw                                   # noqa: E402
from check_geometry import world_geometry, config_mismatches      # noqa: E402

LEGACY = {
    'four_rigid_ground.sdf': ['--n', '4', '--ground-start'],
    'three_rigid_ground.sdf': ['--n', '3', '--ground-start'],
    'two_rigid_ground.sdf': ['--n', '2', '--ground-start'],
    'four_rigid_ground_3915.sdf': ['--n', '4', '--ground-start', '--azimuths', '30,90,150,270'],
    'three_rigid_ground_m10.sdf': ['--n', '3', '--ground-start', '--payload-mass', '1.0'],
    'four_rigid.sdf': ['--n', '4', '--payload-z', '0.6'],
}
RIG = sorted(mcw.WORLDS)


@pytest.mark.parametrize('name', sorted(LEGACY))
def test_generator_defaults_reproduce_the_legacy_worlds(name, tmp_path):
    out = tmp_path / name
    subprocess.run([sys.executable, str(ASSETS / 'generate_rigid_world.py'),
                    '--cable-len', '0.5', '--detachable', *LEGACY[name], '--out', str(out)],
                   check=True, cwd=ASSETS, stdout=subprocess.DEVNULL)
    assert out.read_bytes() == (ASSETS / name).read_bytes()


def test_rig_worlds_and_models_are_up_to_date():
    r = subprocess.run([sys.executable, str(REPO / 'tools/make_carry_worlds.py'), '--check'],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stdout


@pytest.mark.parametrize('name', RIG)
def test_rig_world_reads_back_the_rig_geometry(name):
    g = world_geometry(ASSETS / name)
    args = mcw.WORLDS[name]
    mass = float(args[args.index('--payload-mass') + 1]) if '--payload-mass' in args else 0.86
    for k, v in dict(mcw.EXPECT, load_mass=mass).items():
        assert abs(g[k] - v) < 1e-4, f'{name}: {k} {g[k]:.4f} != {v}'
    assert not [k for k in g if k.startswith('_warn')], g
    # ring inertia from load_mass: outer 0.255, inner 0.195, 40 mm
    assert g['load_ixx'] == pytest.approx(2.227e-2 * mass / 0.86, rel=1e-3)
    assert g['load_izz'] == pytest.approx(4.431e-2 * mass / 0.86, rel=1e-3)


def test_rig_layouts():
    az = {n: world_geometry(ASSETS / n)['attach_azimuths'] for n in RIG}
    assert az['four_rigid_ground_rig.sdf'] == [0.0, 90.0, 180.0, 270.0]
    assert az['four_rigid_ground_rig_3915.sdf'] == [30.0, 90.0, 150.0, 270.0]
    assert az['three_rigid_ground_rig.sdf'] == [0.0, 120.0, 240.0]
    assert world_geometry(ASSETS / 'three_rigid_ground_rig.sdf')['num_drones'] == 3


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
    for name in ('four_rigid_ground.sdf', 'three_rigid_ground.sdf', 'four_rigid.sdf'):
        g = world_geometry(ASSETS / name)
        assert g['pivot_dz'] == pytest.approx(0.0, abs=1e-6)
        assert g['drone_airframe_mass'] == pytest.approx(0.64)
