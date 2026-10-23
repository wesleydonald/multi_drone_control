"""Twin reliability (8 Oct): the runner starts the schedule only once the planner is publishing
references (R1148 pressed ARM 17 s before the planner was up), and stops a run the fleet manager
grounded before TAKEOFF instead of waiting out the duration."""
import os
import sys
import types

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _runner(mode, refs):
    from experiment.runner_node import ExperimentRunner
    f = types.SimpleNamespace(
        sim_t=1.0, pose=[1, 1], cmd=[1, 1], payload=1, ref=refs, n=2, _subs_ok_since=0.0,
        cfg=types.SimpleNamespace(launch_args={'mode': mode} if mode else {}, num_drones=2),
        get_subscriptions_info_by_topic=lambda t: [types.SimpleNamespace(node_name='fleet_manager')])
    f._n_cmd_ready = lambda: 2
    return lambda: ExperimentRunner.stack_ready(f)


@pytest.mark.parametrize('mode, refs, ready', [
    ('dissipative', [None, None], False), ('dissipative', [(0,), None], False),
    ('dissipative', [(0,), (0,)], True), ('mpc', [None, None], False), (None, [None, None], False),
    ('attach', [None, None], False), ('attach', [(0,), (0,)], True), ('free_hover', [None, None], True),
    ('network', [None, None], False), ('network', [(0,), (0,)], True)])
def test_planner_modes_wait_for_references(mode, refs, ready):
    pytest.importorskip('interfaces.msg')
    assert _runner(mode, refs)() is ready


def test_grounded_before_takeoff_stops_the_run():
    pytest.importorskip('interfaces.msg')
    from experiment import runner_node as rn
    t = [4.0]
    f = types.SimpleNamespace(failures=[], _rel=lambda: t[0], _log_event=lambda *a: None,
                              t_partner_release=None, t_grounded=None, stop_reason=None,
                              _ROSOUT=rn.ExperimentRunner._ROSOUT, _planner_lines=[],
                              cfg=types.SimpleNamespace(duration_s=120.0))
    rn.ExperimentRunner._rosout_cb(f, types.SimpleNamespace(
        name='fleet_manager', msg='Drone 2 disarmed before TAKEOFF: fleet disarmed, TAKEOFF refused.'))
    assert f.t_grounded == 4.0
    t[0] = 5.0
    assert rn.ExperimentRunner.finished(f) is False
    t[0] = 6.1
    assert rn.ExperimentRunner.finished(f) is True and 'grounded before TAKEOFF' in f.stop_reason


def test_the_hand_over_and_the_path_start_are_events():
    """Card 2026-10-23_paper_method_compare: the network's hand-over and the path's start after the hover hold are
    events.csv rows on the run clock, from either generator's line (C's scorer windows on them)."""
    pytest.importorskip('interfaces.msg')
    from experiment import runner_node as rn
    ev = []
    f = types.SimpleNamespace(failures=[], _rel=lambda: 12.5, _log_event=lambda *a: ev.append(a),
                              t_partner_release=None, t_grounded=None, _ROSOUT=rn.ExperimentRunner._ROSOUT,
                              _planner_lines=[])
    for line in ('[planner] hover hold 5.0 s before the line_x',
                 '[dissipative] handover OCP -> dissipative network (airborne)',
                 '[planner] hover hold done: line_x starts'):
        rn.ExperimentRunner._rosout_cb(f, types.SimpleNamespace(name='dissipative_planner', msg=line))
    rn.ExperimentRunner._rosout_cb(f, types.SimpleNamespace(name='mpc_planner',
                                                            msg='[planner] hover hold done: fig_8 starts'))
    assert ev == [('NET_HANDOVER', None), ('PATH_START', 'line_x')]


def test_attach_waits_for_the_carriers_references_not_the_newcomers():
    pytest.importorskip('interfaces.msg')
    from experiment.runner_node import ExperimentRunner
    # R1308: attach was not a planner mode here, so the clock started while the planner compiled
    cfg = types.SimpleNamespace(launch_args={'mode': 'attach'}, num_drones=3)
    f = types.SimpleNamespace(cfg=cfg, n=4, sim_t=10.0, pose=[(0, 0, 0)] * 4, cmd=[1] * 4,
                              ref=[None] * 4, payload=(0, 0, 0), _subs_ok_since=None,
                              _n_cmd_ready=lambda: 4,
                              get_subscriptions_info_by_topic=lambda t: [types.SimpleNamespace(node_name='fleet_manager')])
    ready = lambda: ExperimentRunner.stack_ready(f)  # noqa: E731
    assert not ready()                               # the planner is not publishing yet
    f.ref[:3] = [(0.0, 0.0, 0.0)] * 3                # the carriers' references; the newcomer has none
    ready()
    f._subs_ok_since -= 3.0
    assert ready()

