"""The rod in the plan (card docs/experiments/2026-10-31_rod_in_plan.md) cannot disagree with its world: one resolver
(bringup.profiles.rod_model) for the launch, the check and the prebuild, and --check refuses a world whose rods the one
modelled rod is not: another mass or centre of mass, rods that differ, a segmented arm. Nothing is started."""
import os
import sys

import pytest
import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(REPO, 'src', 'bringup'))

from bringup.profiles import rod_model  # noqa: E402
import check_geometry as CG  # noqa: E402
import run_experiment as R  # noqa: E402

ASSETS = os.path.join(REPO, 'simulation_assets')
EXP = os.path.join(REPO, 'configs', 'experiments')
TWIN_ROD = dict(rod_level_mass='0.040', rod_level_com_frac='0.025', cable_len='1.0')   # the 4 Nov links


def test_the_resolver_reads_the_typed_rod_first_then_the_twins():
    assert rod_model({**TWIN_ROD, 'plan_rod_level': 'false'}) == (False, 0.0, 0.0)
    assert rod_model({**TWIN_ROD, 'plan_rod_level': ''}) == (False, 0.0, 0.0)
    assert rod_model({**TWIN_ROD, 'plan_rod_level': 'true'}) == (True, 0.040, pytest.approx(0.025))
    assert rod_model({**TWIN_ROD, 'plan_rod_level': True, 'cable_len': '0.55'}) == (True, 0.040, pytest.approx(0.01375, abs=1e-4))
    assert rod_model({**TWIN_ROD, 'plan_rod_level': 'true', 'rod_mass': '0.08', 'rod_com': '0.3'}) == (True, 0.08, 0.3)
    assert rod_model({'rod_mass': '0.075', 'rod_com': '0.27'}) == (False, 0.075, 0.27)     # R3000: typed, switch off


@pytest.mark.parametrize('off', [{'sil': 'true'}, {'reconfig_mode': 'network'}])
def test_the_switch_stays_off_for_the_sil_plant_and_the_network(off):
    assert rod_model({**TWIN_ROD, 'plan_rod_level': 'true', **off}) == (False, 0.0, 0.0)


@pytest.mark.parametrize('world,com', [
    ('four_rigid_ground_rig_L100.sdf', 0.025), ('four_rigid_ground_rig_L100_p115.sdf', 0.025),
    ('four_rigid_ground_rig.sdf', 0.01375), ('eight_rigid_ground_rig_R50_L100.sdf', 0.025),
    ('nine_rigid_ground_rig_R53_L100.sdf', 0.025), ('three_rigid_ground_rig_attach_L100.sdf', 0.025),
    ('three_rigid_ground_rig_attach.sdf', 0.01375), ('sq_rigid_ground_rig_attach_welded_pickup_L100_p115.sdf', 0.025),
    ('three_rigid_ground_rig_attach_pickup_L100_p115.sdf', 0.025)])
def test_every_rod_of_these_worlds_is_the_one_rod_the_planner_models(world, com):
    g = CG.world_geometry(os.path.join(ASSETS, world))
    rods = g['rods'] + ([g['arm_rod']] if g.get('arm_rod') else [])
    assert rods and all(m == pytest.approx(0.040) and c == pytest.approx(com, abs=1e-3) for m, c in rods)
    L = g['cable_len']
    assert not CG.rod_mismatches(g, {**TWIN_ROD, 'plan_rod_level': 'true', 'cable_len': str(L)})


def test_a_world_the_rod_does_not_fit_comes_out():
    on = {**TWIN_ROD, 'plan_rod_level': 'true'}
    g = CG.world_geometry(os.path.join(ASSETS, 'four_rigid_ground_rig_L100_rod1g.sdf'))
    assert [k for k, _, _ in CG.rod_mismatches(g, on)] == ['rod_mass', 'rod_com']     # 1 g, CoM 0.267 m (the old split)
    assert not CG.rod_mismatches(g, {**on, 'rod_mass': '0.001', 'rod_com': '0.2667'})
    good = CG.world_geometry(os.path.join(ASSETS, 'four_rigid_ground_rig_L100.sdf'))
    assert [k for k, _, _ in CG.rod_mismatches(good, {**on, 'rod_com': '0.3'})] == ['rod_com']
    assert [k for k, _, _ in CG.rod_mismatches(good, {**on, 'cable_len': '0.55'})] == ['rod_com']
    differ = dict(good, rods=[(0.040, 0.025)] * 3 + [(0.05, 0.025)])
    assert [k for k, _, _ in CG.rod_mismatches(differ, on)] == ['rod_mass']
    assert [k for k, _, _ in CG.rod_mismatches(dict(good, arm_rod=(0.0, 0.0)), on)] == ['rod_mass', 'rod_com']
    assert CG.rod_mismatches({}, on)                                  # no rods at all
    assert not CG.rod_mismatches({}, {**on, 'plan_rod_level': 'false'})


def _cfg(tmp_path, base, **args):
    c = yaml.safe_load(open(os.path.join(EXP, base)))
    world = args.pop('_world', None)
    exempt = args.pop('_exempt', None)
    if world:
        c['world'] = world
    if exempt:
        c['geometry_exempt'] = exempt
    c['launch']['args'].update(args)
    p = tmp_path / 'c.yaml'
    p.write_text(yaml.safe_dump(c))
    return str(p)


def test_check_refuses_a_world_the_rod_does_not_fit(tmp_path):
    p = _cfg(tmp_path, 'x1_sqT_ours.yaml', plan_rod_level=True)
    R.checked_config(p)
    with pytest.raises(SystemExit, match='rod_mass: world 0.001, controller 0.04'):
        R.checked_config(_cfg(tmp_path, 'x1_sqT_ours.yaml', plan_rod_level=True, drone_mass=0.55,
                              _world='four_rigid_ground_rig_L100_rod1g.sdf',
                              _exempt={'drone_mass': 'the ballast sits in the airframe'}))
    with pytest.raises(SystemExit, match='rod_com: world 0.025, controller 0.3'):
        R.checked_config(_cfg(tmp_path, 'x1_sqT_ours.yaml', plan_rod_level=True, rod_com=0.3))
    R.checked_config(_cfg(tmp_path, 'x1_sqT_ours.yaml', plan_rod_level=True, rod_com=0.3,
                          _exempt={'rod_com': 'on purpose'}))


def test_check_reads_a_rod_typed_in_the_params_file(tmp_path):
    pf = tmp_path / 'p.yaml'
    pf.write_text('rod_mass: 0.0825\nrod_com: 0.267\nplan_rod_level: true\n')
    with pytest.raises(SystemExit, match='rod_mass: world 0.04, controller 0.0825'):
        R.checked_config(_cfg(tmp_path, 'x1_sqT_ours.yaml', params_file=str(pf)))


def test_the_prebuild_builds_the_rod_the_launch_flies(tmp_path):
    pytest.importorskip('acados_template')
    import prebuild_planner as P
    _, on = P.config_builds(_cfg(tmp_path, 'x1_sqT_ours.yaml', plan_rod_level=True))
    assert on['rod_mass'] == 0.040 and on['rod_com'] == pytest.approx(0.025)
    _, off = P.config_builds(_cfg(tmp_path, 'x1_sqT_ours.yaml', plan_rod_level=False))
    assert off['rod_mass'] == 0.0 and off['rod_com'] == 0.0
    _, net = P.config_builds(_cfg(tmp_path, 'x1_sqT_ours.yaml', plan_rod_level=True, reconfig_mode='network'))
    assert net['rod_mass'] == 0.0
