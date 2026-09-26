"""P2.5 contracts for the M2C-only disarmed external-reference fast path."""

import ast
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np


_METHODS = {
    '_required_external_reference_samples',
    '_handle_disarmed_external_reference_fast_path',
}


class _Logger:
    def warn(self, *args, **kwargs):
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
        name='DummyController', bases=[], keywords=[], body=methods, decorator_list=[]
    )
    module = ast.Module(body=[dummy], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {'np': np, 'time': time, 'MultiDOFJointTrajectory': object}
    exec(compile(module, str(source_path), 'exec'), namespace)

    obj = namespace['DummyController']()
    obj.N = 20
    obj.skip_steps = 3
    obj.disarmed_external_reference_fast_path = True
    obj.armed = False
    obj.external_traj = np.ones((17, 61), dtype=float)
    obj.last_external_reference_time = None
    obj.external_reference_accepted_count = 0
    obj.external_reference_rejected_count = 0
    obj.external_reference_state = 'WAITING'
    obj._test_now_s = time.time()
    obj._node_now_s = lambda: obj._test_now_s
    obj.get_logger = lambda: _Logger()
    return obj


def _point(x=1.0):
    transform = SimpleNamespace(
        translation=SimpleNamespace(x=x, y=2.0, z=3.0),
        rotation=SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0),
    )
    return SimpleNamespace(transforms=[transform])


def test_disarmed_fast_path_accepts_required_reference_without_full_numpy_trajectory():
    controller = _controller_stub()
    msg = SimpleNamespace(points=[_point(float(i)) for i in range(61)])

    handled = controller._handle_disarmed_external_reference_fast_path(msg)

    assert handled is True
    assert controller.external_traj is None
    assert controller.last_external_reference_time is not None
    assert controller.external_reference_accepted_count == 1
    assert controller.external_reference_rejected_count == 0
    assert controller.external_reference_state == 'DISARMED_LIGHTWEIGHT'


def test_fast_path_falls_through_when_armed_so_normal_full_parser_is_used():
    controller = _controller_stub()
    controller.armed = True
    msg = SimpleNamespace(points=[_point(float(i)) for i in range(61)])

    handled = controller._handle_disarmed_external_reference_fast_path(msg)

    assert handled is False
    assert controller.external_reference_accepted_count == 0


def test_disarmed_fast_path_rejects_short_reference_without_claiming_health():
    controller = _controller_stub()
    msg = SimpleNamespace(points=[_point(float(i)) for i in range(60)])

    handled = controller._handle_disarmed_external_reference_fast_path(msg)

    assert handled is True
    assert controller.last_external_reference_time is None
    assert controller.external_reference_accepted_count == 0
    assert controller.external_reference_rejected_count == 1
