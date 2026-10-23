"""The runner's half of rejoin_task and the lapped path (card 2026-10-23_scalability cases 6 and 7): a TASK event is
'drone_id x,y,z,dwell; ...' on /fleet/task, refused before Gazebo starts unless the run has rejoin_departed and
rejoin_task, the drone is a carrier, and every waypoint is inside the planner's staging bound and high enough that the
rod's end clears the floor; the planner's TASK and lap-end lines become events.csv rows."""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from experiment.config import Event, ExperimentConfig, task_arg  # noqa: E402

import yaml  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _doc(task='1 1.45,1.45,1.93,15', **launch):
    args = {'num_drones': 4, 'mode': 'dissipative', 'cable_len': 1.0, 'rejoin_departed': True, 'departed_land': False,
            'rejoin_task': True}
    args.update(launch)
    return {'name': 'tk', 'world': 'four_rigid_ground_rig_L100.sdf',
            'launch': {'package': 'bringup', 'file': 'sim_control_launch.py',
                       'args': {k: v for k, v in args.items() if v is not None}},
            'io_launch': {'file': 'sim_io_launch.py', 'args': {'num_drones': 4, 'tether_tips': True}},
            'fleet': {'num_drones': 4, 'n_total': 4}, 'timing': {'duration_s': 120.0},
            'events': [{'t': 3.0, 'do': 'ARM'}, {'t': 6.0, 'do': 'TAKEOFF'}, {'t': 25.0, 'do': 'DETACH', 'arg': 1},
                       {'t': 35.0, 'do': 'TASK', 'arg': task}, {'t': 70.0, 'do': 'ATTACH', 'arg': 1}]}


def _load(tmp_path, doc):
    p = tmp_path / 'tk.yaml'
    p.write_text(yaml.safe_dump(doc, sort_keys=False))
    return ExperimentConfig.from_yaml(str(p))


def test_task_arg():
    assert task_arg('7 1.45,-1.45,1.93,15') == (7, [(1.45, -1.45, 1.93, 15.0)])
    assert task_arg('3 1,1.4,2,0; -1,1.4,2,5')[1] == [(1.0, 1.4, 2.0, 0.0), (-1.0, 1.4, 2.0, 5.0)]
    for bad in (None, '', '7', 'x 1,2,3,4', '7 1,2,3', '7 1,2,3,-1', '7 1,2,inf,1', '-1 1,2,3,4'):
        with pytest.raises(ValueError, match='TASK arg'):
            Event(1.0, 'TASK', bad)


def test_a_task_config_loads(tmp_path):
    assert _load(tmp_path, _doc()).validate()


@pytest.mark.parametrize('task, launch, why', [
    (None, dict(rejoin_task=None), 'rejoin_departed: true and rejoin_task: true'),
    (None, dict(rejoin_departed=None), 'rejoin_departed: true and rejoin_task: true'),
    ('4 1.45,1.45,1.93,15', {}, 'only a carrier'),
    ('1 1.6,1.45,1.93,15', {}, 'outside the staging bound'),
    ('1 1.45,1.45,1.3,15', {}, 'its end would near the floor'),
    ('1 1.45,1.45,1.6,15', dict(cable_len=1.5), 'its end would near the floor'),
    ('1 1.45,1.4,1.93,15', dict(pos_limit_m=1.5), 'outside the staging bound'),     # a typed limit's bound, 1.32 m
])
def test_a_task_that_cannot_fly_is_refused_before_the_run(tmp_path, task, launch, why):
    with pytest.raises(ValueError, match=why):
        _load(tmp_path, _doc(task=task or '1 1.45,1.45,1.93,15', **launch))


def test_the_shipped_case_configs():
    def load(name):
        return ExperimentConfig.from_yaml(os.path.join(REPO, 'configs', 'experiments', name + '.yaml'))
    c6 = load('s8_chain4task_R50_L100_it5')
    assert [task_arg(e.arg)[0] for e in c6.events if e.do == 'TASK'] == [7, 3, 5, 1]
    assert [e.arg for e in c6.events if e.do == 'ATTACH'] == [7, 3, 5, 1]
    assert c6.launch_args['rejoin_task'] is True and c6.launch_args['rejoin_departed'] is True
    for name in ('s8_fig8laps_h5_v200_R50_L100_it5', 's8_chain4back_fig8laps_R50_L100_it5'):
        c7 = load(name)
        assert (c7.launch_args['traj_laps'], c7.launch_args['traj_lap_hold_s']) == (9, 3.0)
        assert c7.events[-2].do == 'WAIT_TRAJ' and c7.events[-2].arg.startswith('lap 9')


def test_the_runner_publishes_a_task_and_logs_the_task_and_lap_lines():
    pytest.importorskip('interfaces.msg')
    from experiment import runner_node as rn
    sent, rows = [], []
    f = types.SimpleNamespace(failures=[], _rel=lambda: 4.0, _log_event=lambda *a: rows.append(a),
                              t_partner_release=None, _ROSOUT=rn.ExperimentRunner._ROSOUT, _planner_lines=[],
                              get_logger=lambda: types.SimpleNamespace(warn=lambda *a, **k: None),
                              task_pub=types.SimpleNamespace(topic_name='/fleet/task', publish=sent.append,
                                                             get_subscription_count=lambda: 1))
    f._publish = types.MethodType(rn.ExperimentRunner._publish, f)
    rn.ExperimentRunner._fire(f, Event(5.0, 'TASK', '7 1.45,-1.45,1.93,15'))
    assert [m.data for m in sent] == ['7 1.45,-1.45,1.93,15'] and rows[-1] == ('TASK', '7 1.45,-1.45,1.93,15')
    rows.clear()
    for text in ('[dissipative] TASK START drone 8 (/drone_7): (1.45, -1.45, 1.93) 15.0 s, at 0.20 m/s',
                 '[dissipative] TASK AT drone 8 (/drone_7) waypoint 1 of 1 (1.45, -1.45, 1.93) 6.1 s after its TASK',
                 '[dissipative] TASK DONE drone 8 (/drone_7): back at its staging point 27.3 s after its TASK START',
                 '[dissipative] TASK REFUSED drone 4 (/drone_3): it is in its approach',
                 "[dissipative] TASK REFUSED '7 1,2': waypoint '1,2': want x,y,z,dwell, finite, dwell >= 0",
                 '[dissipative] TASK ABORT drone 2 (/drone_1) (LAND): up to 1.93 m clear of the ring',
                 '[dissipative] lap 5 done at t=96.25 s: trajectory held 3.0 s (traj_lap_hold_s; 4 of 9 laps to fly)'):
        rn.ExperimentRunner._rosout_cb(f, types.SimpleNamespace(name='dissipative_planner', msg=text))
    assert rows == [('TASK_START', '7'), ('TASK_AT', '7'), ('TASK_DONE', '7'), ('TASK_REFUSED', '3'),
                    ('TASK_REFUSED', None), ('TASK_ABORT', '1'), ('LAP_DONE', '5')]
