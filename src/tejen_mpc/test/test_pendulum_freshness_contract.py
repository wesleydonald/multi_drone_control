"""Regression tests for the payload pendulum-state freshness contract.

These tests extract only the pure Controller helpers and inspect the deployed
source so they run without ROS/acados in ordinary pytest.
"""

import ast
from pathlib import Path
import time

import numpy as np


_METHODS = {
    '_pendulum_state_fault_reason',
    '_latch_pendulum_state_fault',
}


class _Logger:
    def error(self, *args, **kwargs):
        pass


def _controller_stub():
    source_path = Path(__file__).resolve().parents[1] / 'tejen_mpc' / 'main.py'
    tree = ast.parse(source_path.read_text())
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'Controller'
    )
    methods = [
        node for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name in _METHODS
    ]
    assert {node.name for node in methods} == _METHODS

    dummy = ast.ClassDef(
        name='DummyController',
        bases=[],
        keywords=[],
        body=methods,
        decorator_list=[],
    )
    module = ast.Module(body=[dummy], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {'np': np, 'time': time}
    exec(compile(module, str(source_path), 'exec'), namespace)

    obj = namespace['DummyController']()
    obj.pendulum_state_timeout_s = 0.25
    obj.pendulum_state = np.zeros(4, dtype=float)
    obj.last_pendulum_update_time = None
    obj.pendulum_state_age_s = float('nan')
    obj.pendulum_state_fault_latched = False
    obj.pendulum_state_fault_reason = ''
    obj.pendulum_state_fault_latch_count = 0
    obj.get_logger = lambda: _Logger()
    return obj


def test_missing_pendulum_state_is_invalid():
    controller = _controller_stub()
    reason = controller._pendulum_state_fault_reason(now_s=time.monotonic())
    assert reason == 'missing pendulum state'
    assert np.isinf(controller.pendulum_state_age_s)


def test_fresh_pendulum_state_is_valid():
    controller = _controller_stub()
    now = time.monotonic()
    controller.last_pendulum_update_time = now - 0.10
    reason = controller._pendulum_state_fault_reason(now_s=now)
    assert reason is None
    assert 0.09 <= controller.pendulum_state_age_s <= 0.11


def test_stale_or_nonfinite_pendulum_state_is_invalid():
    controller = _controller_stub()
    now = time.monotonic()
    controller.last_pendulum_update_time = now - 0.30
    assert 'stale' in controller._pendulum_state_fault_reason(now_s=now)

    controller.last_pendulum_update_time = now
    controller.pendulum_state[2] = np.nan
    assert 'NaN/Inf' in controller._pendulum_state_fault_reason(now_s=now)


def test_airborne_fault_latch_is_diagnostic_only_and_persistent():
    controller = _controller_stub()
    controller._latch_pendulum_state_fault('pendulum state stale (0.300 s)')
    controller._latch_pendulum_state_fault('different reason')

    assert controller.pendulum_state_fault_latched is True
    assert controller.pendulum_state_fault_reason == 'pendulum state stale (0.300 s)'
    assert controller.pendulum_state_fault_latch_count == 1

    source_path = Path(__file__).resolve().parents[1] / 'tejen_mpc' / 'main.py'
    source = source_path.read_text()
    tree = ast.parse(source)
    controller_class = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'Controller'
    )
    latch = next(
        node for node in controller_class.body
        if isinstance(node, ast.FunctionDef) and node.name == '_latch_pendulum_state_fault'
    )
    called_attrs = {
        node.func.attr
        for node in ast.walk(latch)
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
    }
    assert '_make_stationary_external_reference' not in called_attrs
    assert '_latch_external_reference_fault' not in called_attrs


def test_control_loop_rejects_invalid_pendulum_before_first_takeoff_solve():
    source_path = Path(__file__).resolve().parents[1] / 'tejen_mpc' / 'main.py'
    tree = ast.parse(source_path.read_text())
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'Controller'
    )
    control_loop = next(
        node for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name == 'control_loop'
    )

    calls = [
        (getattr(node.func, 'attr', None), getattr(node, 'lineno', 0))
        for node in ast.walk(control_loop)
        if isinstance(node, ast.Call)
    ]
    pendulum_lines = [line for name, line in calls if name == '_pendulum_state_fault_reason']
    reference_lines = [line for name, line in calls if name == 'get_reference_trajectory']
    assert pendulum_lines and reference_lines
    assert min(pendulum_lines) < min(reference_lines)

    source = ast.get_source_segment(source_path.read_text(), control_loop)
    assert 'self.takeoff_requested = False' in source
    assert 'len(self.control_history) == 0' in source
    assert '_latch_pendulum_state_fault' in source


def test_pendulum_callback_uses_node_physical_clock():
    source_path = (
        Path(__file__).resolve().parents[2]
        / 'tejen_utility_objects' / 'tejen_utility_objects' / 'callback_manager.py'
    )
    source = source_path.read_text()
    tree = ast.parse(source)
    manager = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'CallbackManager'
    )
    callback = next(
        node for node in manager.body
        if isinstance(node, ast.FunctionDef) and node.name == 'pendulum_callback'
    )
    callback_source = ast.get_source_segment(source, callback)
    assert 'self.node.get_clock().now().nanoseconds * 1e-9' in callback_source
