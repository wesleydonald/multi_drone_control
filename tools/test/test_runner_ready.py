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
        cfg=types.SimpleNamespace(launch_args={'mode': mode} if mode else {}),
        get_subscriptions_info_by_topic=lambda t: [types.SimpleNamespace(node_name='fleet_manager')])
    f._n_cmd_ready = lambda: 2
    return lambda: ExperimentRunner.stack_ready(f)


@pytest.mark.parametrize('mode, refs, ready', [
    ('dissipative', [None, None], False), ('dissipative', [(0,), None], False),
    ('dissipative', [(0,), (0,)], True), ('mpc', [None, None], False), (None, [None, None], False),
    ('attach', [None, None], True), ('free_hover', [None, None], True)])
def test_planner_modes_wait_for_references(mode, refs, ready):
    pytest.importorskip('interfaces.msg')
    assert _runner(mode, refs)() is ready


def test_grounded_before_takeoff_stops_the_run():
    pytest.importorskip('interfaces.msg')
    from experiment import runner_node as rn
    t = [4.0]
    f = types.SimpleNamespace(failures=[], _rel=lambda: t[0], _log_event=lambda *a: None,
                              t_partner_release=None, t_grounded=None, stop_reason=None,
                              _ROSOUT=rn.ExperimentRunner._ROSOUT,
                              cfg=types.SimpleNamespace(duration_s=120.0))
    rn.ExperimentRunner._rosout_cb(f, types.SimpleNamespace(
        name='fleet_manager', msg='Drone 2 disarmed before TAKEOFF: fleet disarmed, TAKEOFF refused.'))
    assert f.t_grounded == 4.0
    t[0] = 5.0
    assert rn.ExperimentRunner.finished(f) is False
    t[0] = 6.1
    assert rn.ExperimentRunner.finished(f) is True and 'grounded before TAKEOFF' in f.stop_reason
