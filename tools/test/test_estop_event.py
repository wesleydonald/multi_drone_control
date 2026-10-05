"""ESTOP runner event (mux abort latch card 2026-09-28, v3 item 5): the config accepts it
only on an abort arm, the runner sends the exact string the fleet manager parses, and the
arm-C config is the canonical M1 plus an ESTOP after the partner release."""
import os
import sys
import types

import pytest
import yaml

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)

from experiment.config import ESTOP_STAY_DOWN_S, Event, ExperimentConfig  # noqa: E402

CFG = os.path.join(REPO, 'configs', 'experiments')


def _doc(**timing):
    return {'name': 'estop_t', 'launch': {'args': {'num_drones': 3}},
            'fleet': {'num_drones': 3, 'n_total': 3},
            'timing': dict({'duration_s': 60.0}, **timing),
            'events': [{'t': 3, 'do': 'ARM'}, {'t': 5, 'do': 'TAKEOFF'},
                       {'t': 20, 'do': 'estop'}],
            'criteria': {'forbid_abort': False}}


def _load(tmp_path, doc):
    p = tmp_path / 'c.yaml'
    p.write_text(yaml.safe_dump(doc))
    return ExperimentConfig.from_yaml(str(p))


def test_estop_is_a_known_event(tmp_path):
    c = _load(tmp_path, _doc(stop_after_abort_s=10.0))
    assert [e.do for e in c.events][-1] == 'ESTOP'


@pytest.mark.parametrize('criteria', [None, {'forbid_abort': True}, {}])
def test_estop_needs_forbid_abort_false(tmp_path, criteria):
    doc = _doc(stop_after_abort_s=10.0)
    if criteria is None:
        del doc['criteria']
    else:
        doc['criteria'] = criteria
    with pytest.raises(ValueError, match='forbid_abort'):
        _load(tmp_path, doc)


def test_estop_needs_the_stay_down_window(tmp_path):
    with pytest.raises(ValueError, match='stay-down'):
        _load(tmp_path, _doc(stop_after_abort_s=ESTOP_STAY_DOWN_S - 1.0))
    assert _load(tmp_path, _doc(stop_after_abort_s=0.0))      # 0 = record the full duration


def test_runner_sends_the_managers_estop_string():
    pytest.importorskip('interfaces.msg')       # runner_node imports ROS messages
    from experiment.runner_node import ExperimentRunner
    sent = []
    f = types.SimpleNamespace(
        fleet_pub=object(), _rel=lambda: 1.0, _log_event=lambda *a: None,
        get_logger=lambda: types.SimpleNamespace(warn=lambda *a: None),
        _publish=lambda pub, msg, what, exclude_self=False: sent.append((pub, msg.data, what)))
    ExperimentRunner._fire(f, Event(1.0, 'ESTOP'))
    assert sent == [(f.fleet_pub, 'ESTOP', '/fleet/command ESTOP')]
    # the manager's parser: strip().upper() == "ESTOP"
    assert sent[0][1].strip().upper() == 'ESTOP'


def test_arm_c_config_is_canonical_plus_estop():
    base = yaml.safe_load(open(os.path.join(CFG, 'partner_attached_orbit.yaml')))
    arm = yaml.safe_load(open(os.path.join(CFG, 'partner_attached_orbit_estop.yaml')))
    assert arm['launch'] == base['launch'] and arm['world'] == base['world']
    assert arm['io_launch'] == base['io_launch'] and arm['fleet'] == base['fleet']
    c = ExperimentConfig.from_yaml(os.path.join(CFG, 'partner_attached_orbit_estop.yaml'))
    order = [e.do for e in sorted(c.events, key=lambda e: e.t)]
    assert order.index('ESTOP') == order.index('WAIT_PARTNER_RELEASE') + 1
    assert order.index('DETACH') < order.index('WAIT_PARTNER_RELEASE')
    assert 'LAND' not in order and 'WAIT_REWELD' not in order
    assert c.criteria.forbid_abort is False and c.stop_after_abort_s == 10.0


def test_runner_fails_the_run_on_a_manager_arm_failure():
    """Q6b: a failed or refused ARM raises no /fleet/abort, so the runner reads the log."""
    pytest.importorskip('interfaces.msg')
    from experiment import runner_node as rn
    f = types.SimpleNamespace(failures=[], _rel=lambda: 4.0, _log_event=lambda *a: None,
                              t_partner_release=None, _ROSOUT=rn.ExperimentRunner._ROSOUT)
    log = lambda name, text: types.SimpleNamespace(name=name, msg=text)  # noqa: E731
    rn.ExperimentRunner._rosout_cb(f, log('fleet_manager', 'Arming all drones...'))
    assert f.failures == []
    rn.ExperimentRunner._rosout_cb(
        f, log('fleet_manager', 'ARM FAILED for drone(s) [2]: disarming the fleet'))
    rn.ExperimentRunner._rosout_cb(
        f, log('fleet_manager', 'ARM REFUSED: mux latched on drone 3 (relaunch the muxes)'))
    assert len(f.failures) == 1 and f.failures[0].startswith(rn.ARM_FAIL)
    rn.ExperimentRunner._rosout_cb(f, log('dissipative_planner', 'ARM FAILED'))
    assert len(f.failures) == 1
