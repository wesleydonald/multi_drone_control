"""Passive observer runs beside legacy integral without changing control inputs.

Ported from his 26ff38be (node part only; his M1/M2 IRL launch and runner wiring, and its test, are not carried).
"""
import ast
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mpc.lateral_disturbance_observer import (
    LateralDisturbanceObserver, nominal_lateral_acceleration, parse_xy_bias_mode,
)

ROOT = Path(__file__).resolve().parents[3]
MAIN = ROOT / 'src/tejen_mpc/tejen_mpc/main.py'


def controller(mode='legacy_integral', diagnostic=False):
    tree = ast.parse(MAIN.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controller')
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    names = {'update_lateral_disturbance_observer', '_lateral_disturbance_model_is_valid',
             'update_xy_integral_state'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    module = ast.Module(body=[ast.ClassDef(name='Probe', bases=[], keywords=[],
                                         body=methods, decorator_list=[])], type_ignores=[])
    ns = dict(np=np, DT=1/30, LateralDisturbanceObserver=LateralDisturbanceObserver,
              parse_xy_bias_mode=parse_xy_bias_mode, nominal_lateral_acceleration=nominal_lateral_acceleration)
    exec(compile(ast.fix_missing_locations(module), str(MAIN), 'exec'), ns)
    p = ns['Probe']()
    values = dict(xy_bias_mode=mode, lateral_disturbance_diagnostics_enabled=diagnostic,
                  xy_integral_limit=.45)
    p.declare_parameter = lambda key, default: SimpleNamespace(value=values.get(key, default))
    block = []
    active = False
    for n in init.body:
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and
                t.id == 'legacy_integral_enabled_param' for t in n.targets):
            active = True
        if active:
            block.append(n)
        if isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and
                t.id == 'integral_terminal_weight' for t in n.targets):
            break
    ns['self'] = p
    exec(compile(ast.fix_missing_locations(ast.Module(body=block, type_ignores=[])), str(MAIN), 'exec'), ns)
    p.cost_weights = (ns['integral_running_weight'], ns['integral_terminal_weight'])
    p.current_pose = np.array([0, 0, 1, np.cos(.02), 0, np.sin(.02), 0, 0, 0, 0])
    p.est_params = np.array([22.])
    p.control_history = [np.array([0, 0, 9.81/22, 0])]
    p.armed = p.takeoff_requested = True
    p.lateral_disturbance_initial_z = 0.
    p.now = 100.
    p._node_now_s = lambda: p.now
    return p


def test_diagnostic_observes_bias_but_keeps_legacy_integral_and_mpc_input_identical():
    normal, passive = controller(), controller(diagnostic=True)
    assert passive.enable_lateral_disturbance_observer
    assert passive.enable_xy_integral_action and not passive.enable_lateral_disturbance_compensation
    assert normal.cost_weights == passive.cost_weights == (15., 30.)
    trajectory = np.zeros((17, 61)); trajectory[0, :] = .04; trajectory[3, :] = 1.
    for _ in range(120):
        normal.now += 1/30; passive.now += 1/30
        for p in (normal, passive):
            p.update_xy_integral_state(trajectory, 0)
            p.update_lateral_disturbance_observer()
        np.testing.assert_array_equal(normal.xy_integral_error, passive.xy_integral_error)
        np.testing.assert_array_equal(normal.xy_integral_saturated, passive.xy_integral_saturated)
    assert passive.lateral_disturbance_update_count > 0
    assert abs(passive.lateral_disturbance_estimate[0]) > .01
    # Execute the actual control-loop selection, not a parallel test formula.
    tree = ast.parse(MAIN.read_text())
    assignment = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign) and
                      any(isinstance(t, ast.Name) and t.id == 'lateral_disturbance_for_mpc'
                          for t in n.targets))
    for p in (normal, passive):
        ns = {'self': p, 'np': np}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[assignment], type_ignores=[])), str(MAIN), 'exec'), ns)
        np.testing.assert_array_equal(ns['lateral_disturbance_for_mpc'], np.zeros(2))


@pytest.mark.parametrize('mode,integral,observer,compensation', [
    ('legacy_integral', True, False, False), ('none', False, False, False),
    ('lateral_disturbance_shadow', False, True, False), ('lateral_disturbance', False, True, True),
])
def test_existing_modes_and_default_remain_unchanged(mode, integral, observer, compensation):
    p = controller(mode)
    assert p.enable_xy_integral_action is integral
    assert p.enable_lateral_disturbance_observer is observer
    assert p.enable_lateral_disturbance_compensation is compensation

