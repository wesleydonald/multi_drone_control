"""exit_handoff:=moving in partner_mission.launch.py (docs/rig/cpp_exit_impl_contract_2026-11-15.md 3.10): typed only and
refused unless return_after_drop:=cpp; it sets his planner's switch and starting values and gives his backend its
exit-only context (the prefix topic, 5 deg, the +-1.8 m workspace to 3.26 m, the 3.2 m search top), keeps
require_payload_attached_for_enable true, holds the clearance at 2.9 m and lets max_body_z reach 3.2 m on this line
only. Untyped, the launch is today's (the flown lines dump as the pristine tree's)."""
import math
from pathlib import Path

import pytest
import yaml
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters

from test_partner_mission_geometry import _context
from test_partner_mission_instances import BASE, LAUNCH, _dump
from test_partner_mission_rig import _rig, _s

# 13 Nov's drop-off line (rig_fm9h8.sh, return_after_drop as the sheet types it) and its twin
RIG = dict(minimum_search_z_m='2.20', magnet_drop_below_quad='1.031', ring_cable_len='1.0', ring_z='1.0',
           geometry='rig_m2a', mission_mode='delivery_rejoin', transit='low', max_body_z='2.8')
SIM = dict(minimum_search_z_m='2.20', mission_mode='delivery_rejoin', ring_z='1.0', transit='low')


def _graph(real, **typed):
    if real:
        mod, ctx, ents = _rig(**{**RIG, **typed})
    else:
        typed = {**SIM, **typed}
        mod, ctx = _context('rig_twin_L100', **typed)
        ctx.launch_configurations[mod.TYPED_KEY] = ','.join(typed)
        ents = mod._nodes(ctx)
    planner = next(e for e in ents if isinstance(e, Node) and e._Node__node_name == 'online_join_planner')
    params = {}
    for p in evaluate_parameters(ctx, planner._Node__parameters or []):
        if isinstance(p, dict):
            params.update(p)
    include = next(e for e in ents if isinstance(e, IncludeLaunchDescription))
    args = {_s(ctx, k): _s(ctx, v) for k, v in include.launch_arguments}
    backend = {}
    for f in args['extra_params_file'].split(','):
        backend.update(yaml.safe_load(Path(f).read_text())[mod.BACKEND_NODE]['ros__parameters'])
    return mod, params, backend


@pytest.mark.parametrize('real', [True, False])
def test_untyped_nothing_of_the_exit_reaches_either_node(real):
    mod, params, backend = _graph(real, return_after_drop='cpp')
    assert not set(mod.EXIT_PLANNER) & set(params)
    assert not set(mod.EXIT_BACKEND) & set(backend)
    assert backend['require_payload_attached_for_enable'] is False      # today's C++ return: no object wait


@pytest.mark.parametrize('real', [True, False])
@pytest.mark.parametrize('ret', ['', 'direct'])
def test_moving_is_refused_without_the_cpp_return(real, ret):
    with pytest.raises(RuntimeError, match='exit_handoff:=moving needs return_after_drop:=cpp'):
        _graph(real, exit_handoff='moving', return_after_drop=ret, max_body_z='3.2')


@pytest.mark.parametrize('real', [True, False])
def test_moving_sets_his_planner_and_holds_the_clearance_at_2_9(real):
    mod, params, _ = _graph(real, exit_handoff='moving', return_after_drop='cpp', max_body_z='3.2')
    assert {k: params[k] for k in mod.EXIT_PLANNER} == mod.EXIT_PLANNER
    assert params['c1f2_exit_moving_handoff'] is True and params['at_start_land_after_s'] == 2.0
    hold = 1.0 + mod.DROP_OFFSET_Z + params['drop_clear_height'] + float(params.get('magnet_drop_below_quad', 1.031))
    assert hold == pytest.approx(2.9)


@pytest.mark.parametrize('real', [True, False])
def test_moving_gives_his_backend_its_exit_context_and_keeps_the_object_wait(real):
    mod, params, backend = _graph(real, exit_handoff='moving', return_after_drop='cpp', max_body_z='3.2')
    assert backend['moving_prefix_topic'] == params['exit_prefix_topic'] == '/dynamic_planner/transfer_prefix'
    assert backend['exit_max_swing_angle_rad'] == pytest.approx(math.radians(5.0))
    assert backend['exit_workspace_min'] == [-1.8, -1.8, 0.0] and backend['exit_workspace_max'] == [1.8, 1.8, 3.26]
    assert backend['exit_max_search_z_m'] == 3.2 and backend['exit_protected_s'] == params['exit_protected_s']
    assert backend['require_payload_attached_for_enable'] is True
    assert backend['static_target_topic'] == '/dynamic_planner/static_target'


def test_max_body_z_reaches_3_2_on_the_moving_exit_line_only():
    with pytest.raises(RuntimeError, match=r'outside \[2.6, 3.0\]'):
        _graph(True, return_after_drop='cpp', max_body_z='3.2')
    with pytest.raises(RuntimeError, match='above 2.8 m'):                       # the hold + overshoot is 3.1 m
        _graph(True, exit_handoff='moving', return_after_drop='cpp')
    _graph(True, exit_handoff='moving', return_after_drop='cpp', max_body_z='3.2')
    with pytest.raises(RuntimeError, match=r'outside \[2.6, 3.2\]'):
        _graph(True, exit_handoff='moving', return_after_drop='cpp', max_body_z='3.3')


def test_the_exit_topics_are_his_planners_instance_topics():
    mod, _, _ = _graph(True, return_after_drop='cpp')
    assert mod.PLANNER_TOPICS['exit_prefix_topic'] == mod.EXIT_PREFIX_TOPIC
    assert mod.PLANNER_TOPICS['exit_ring_commitment_topic'] == mod.RING_TOPICS['payload_committed_trajectory_topic']


@pytest.mark.skipif(not BASE.exists(), reason=f'no pristine tree at {BASE} (MDC_424_BASE)')
@pytest.mark.parametrize('typed', [dict(real='true', contact_attach='true', contact_target_z='0.0491', **RIG,
                                        return_after_drop='cpp', mpc_thrust_ratio='22'),
                                   dict(SIM, minimum_search_z_m='1.70', return_after_drop='cpp')],
                         ids=['rig', 'twin'])
def test_untyped_the_cpp_return_lines_dump_as_the_pristine_tree(typed):
    """Except, on the twin's return line, his supervisor's success keys (v3 review: success at the start's landing)."""
    keys = ('success_phase', 'success_after_phase', 'require_handoff_ready')

    def without_supervisor_success(dump):
        out = []
        for e in dump:
            if e[0] == 'node' and e[3] == 'simulation_test_supervisor':
                params = [(k, {n: v for n, v in d.items() if n not in keys}) if k == 'dict' else (k, d)
                          for k, d in e[5]]
                e = e[:5] + (params,) + e[6:]
            out.append(e)
        return out
    now, base = _dump(LAUNCH, typed), _dump(BASE, typed)
    assert without_supervisor_success(now) == without_supervisor_success(base)


@pytest.mark.parametrize('real', [True, False])
def test_the_general_body_limit_bounds_the_loaded_search_on_the_moving_line(real):
    """Review 15 Nov: the trial's 3.2 m body limit covers the loaded approach; cpp_max_z above it is refused, untyped it
    is the limit; the exit's own search top is the same limit; the frozen lines keep cpp_max_z as typed."""
    line = dict(exit_handoff='moving', return_after_drop='cpp', max_body_z='3.2')
    with pytest.raises(RuntimeError, match='cpp_max_z 3.3 is above max_body_z 3.2'):
        _graph(real, cpp_max_z='3.3', **line)
    _, _, backend = _graph(real, **line)
    assert backend['maximum_search_z_m'] == 3.2 and backend['exit_max_search_z_m'] == 3.2
    _, _, backend = _graph(real, cpp_max_z='3.0', **line)
    assert backend['maximum_search_z_m'] == 3.0
    _, _, backend = _graph(real, cpp_max_z='3.3', return_after_drop='cpp')
    assert backend['maximum_search_z_m'] == 3.3 and 'exit_max_search_z_m' not in backend


def _supervisor_params(**typed):
    typed = {**SIM, **typed}
    mod, ctx = _context('rig_twin_L100', **typed)
    ctx.launch_configurations[mod.TYPED_KEY] = ','.join(typed)
    sup = next(e for e in mod._nodes(ctx)
               if isinstance(e, Node) and e._Node__node_name == 'simulation_test_supervisor')
    params = {}
    for p in evaluate_parameters(ctx, sup._Node__parameters or []):
        if isinstance(p, dict):
            params.update(p)
    return params


def test_the_supervisor_status_timeout_is_a_per_run_twin_override():
    """Te-Jen and Codex 15 Nov: the timeout check freezes his backend, so his twin supervisor's backend-status limit
    is typed for that run only; untyped it is 15 s; the rig refuses it."""
    assert _supervisor_params()['planner_status_timeout_s'] == 15.0
    assert _supervisor_params(supervisor_status_timeout_s='120')['planner_status_timeout_s'] == 120.0
    with pytest.raises(RuntimeError, match='supervisor_status_timeout_s is a twin check only'):
        _graph(True, supervisor_status_timeout_s='120')


def test_the_twin_supervisor_passes_a_return_line_at_its_start_only():
    """Te-Jen and Codex 15 Nov (v3 review): on return_after_drop twin lines the supervisor's success is LANDED_DISARMED
    after AT_START; every other twin line keeps our weld as the success."""
    ret = _supervisor_params(return_after_drop='cpp')
    assert (ret['success_phase'], ret['success_after_phase'], ret['require_handoff_ready']) == \
        ('LANDED_DISARMED', 'AT_START', False)
    plain = _supervisor_params()
    assert plain['success_phase'] == 'NONE_PARTNER_WELD_OWNS_SUCCESS' and 'success_after_phase' not in plain
    assert plain['require_handoff_ready'] is True and plain['failure_phase'] == 'LANDED_DISARMED'
