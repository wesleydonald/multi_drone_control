"""The runner's half of rejoin_departed (card 2026-10-23_rejoin_departed): an ATTACH of a carrier's id is a departed
carrier's rejoin, refused before Gazebo starts unless the run has the switch and what the rejoin needs (departed_land
false, the OCP resize, the twin's mode:=dissipative and each rod's end from tether_tips); the planner's REJOIN lines
become events.csv rows. A newcomer's ATTACH (a reserved id) is unchanged."""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from experiment.config import ExperimentConfig  # noqa: E402

import yaml  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _doc(**launch):
    args = {'num_drones': 4, 'mode': 'dissipative', 'rejoin_departed': True, 'departed_land': False}
    args.update(launch)
    return {'name': 'rj', 'world': 'four_rigid_ground_rig_L100.sdf',
            'launch': {'package': 'bringup', 'file': 'sim_control_launch.py',
                       'args': {k: v for k, v in args.items() if v is not None}},
            'io_launch': {'file': 'sim_io_launch.py', 'args': {'num_drones': 4, 'tether_tips': True}},
            'fleet': {'num_drones': 4, 'n_total': 4}, 'timing': {'duration_s': 120.0},
            'events': [{'t': 3.0, 'do': 'ARM'}, {'t': 6.0, 'do': 'TAKEOFF'}, {'t': 25.0, 'do': 'DETACH', 'arg': 1},
                       {'t': 45.0, 'do': 'ATTACH', 'arg': 1}]}


def _load(tmp_path, doc):
    p = tmp_path / 'rj.yaml'
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    return ExperimentConfig.from_yaml(str(p))


def test_a_rejoin_config_loads(tmp_path):
    assert _load(tmp_path, _doc()).validate()


@pytest.mark.parametrize('launch, io, why', [
    (dict(rejoin_departed=None), None, 'rejoins only with rejoin_departed'),
    (dict(departed_land=None), None, 'departed_land: false'),
    (dict(departed_land=True), None, 'departed_land: false'),
    (dict(reconfig_mode='network'), None, 'reconfig_mode ocp'),
    (dict(mode='attach'), None, 'mode: dissipative'),
    ({}, {'num_drones': 4}, 'tether_tips'),
])
def test_a_rejoin_without_what_it_needs_is_refused_before_the_run(tmp_path, launch, io, why):
    doc = _doc(**launch)
    if io is not None:
        doc['io_launch']['args'] = io
    with pytest.raises(ValueError, match=why):
        _load(tmp_path, doc)


def test_the_switch_and_departed_land_may_come_from_the_params_file(tmp_path):
    pf = tmp_path / 'rj_params.yaml'
    pf.write_text(yaml.safe_dump({'fleet_resize': {'rejoin_departed': True, 'departed_land': False}}))
    assert _load(tmp_path, _doc(rejoin_departed=None, departed_land=None, params_file=str(pf))).validate()


def test_a_newcomers_attach_needs_no_switch(tmp_path):
    doc = _doc(rejoin_departed=None, departed_land=None, reserved_attach=1)
    doc['fleet'] = {'num_drones': 4, 'n_total': 5}
    doc['events'][-1]['arg'] = 4
    assert _load(tmp_path, doc).validate()


def test_the_shipped_rejoin_smoke_config_is_a_rejoin():
    c = ExperimentConfig.from_yaml(os.path.join(REPO, 'configs', 'experiments', 'rj4_softdetach9_rejoin_L100.yaml'))
    assert [e.arg for e in c.events if e.do == 'ATTACH'] == [1] and c.launch_args['rejoin_departed'] is True


def test_the_runner_logs_the_rejoin_lines_as_events():
    pytest.importorskip('interfaces.msg')
    from experiment import runner_node as rn
    rows = []
    f = types.SimpleNamespace(failures=[], _rel=lambda: 4.0, _log_event=lambda *a: rows.append(a),
                              t_partner_release=None, _ROSOUT=rn.ExperimentRunner._ROSOUT, _planner_lines=[])
    for text in ('[dissipative] REJOIN START drone 2 (/drone_1): from [0. 1.03 1.93] back to plate 9',
                 '[dissipative] rejoin: drone 2 (/drone_1)\'s rod end 2.1 cm from its plate point',
                 '[dissipative] REJOIN ON PLATE drone 2 (/drone_1): its rod end 0.4 cm from plate 9, 0.3 cm/s',
                 "[dissipative] the twin's tether joint of drone 2 (/drone_1): attached",
                 '[dissipative] REJOIN WELD drone 2 (/drone_1): its tether joint attached 0.10 s after /drone_1/attach',
                 '[dissipative] ATTACH drone 1 (OCP resize, rejoin): n=4, weld at 270 deg cw',
                 '[dissipative] REJOIN BLEND OVER drone 2 (/drone_1): its rod end at most 0.6 cm from its plate point',
                 '[dissipative] REJOIN ABORT drone 4 (/drone_3) (LAND): up to 2.31 m clear of the ring',
                 '[dissipative] REJOIN REFUSED drone 4 (/drone_3): drone 2 (/drone_1) is rejoining (one at a time)',
                 "[dissipative] the twin's tether joint of drone 4 (/drone_3): detached",
                 '[dissipative] staged drone 4 (/drone_3) leaves (LAND): up to 1.93 m clear of the ring'):
        rn.ExperimentRunner._rosout_cb(f, types.SimpleNamespace(name='dissipative_planner', msg=text))
    assert rows == [('REJOIN_START', '1'), ('REJOIN_ON_PLATE', '1'), ('JOINT_ATTACHED', '1'), ('REJOIN_WELD', '1'),
                    ('FOLD_IN', '1'), ('REJOIN_BLEND_OVER', '1'), ('REJOIN_ABORT', '3'), ('REJOIN_REFUSED', '3'),
                    ('JOINT_DETACHED', '3'), ('STAGED_LEAVES', '3')]
