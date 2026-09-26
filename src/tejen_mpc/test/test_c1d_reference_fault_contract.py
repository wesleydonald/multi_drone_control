"""C.1d regression tests for the external-reference fault contract.

These tests deliberately extract only the pure Controller methods under test from
main.py instead of importing the ROS/acados module. That keeps the regression
runnable in ordinary pytest while still exercising the source that is deployed.
"""

import ast
from pathlib import Path
import time

import numpy as np


_METHODS = {
    '_required_external_reference_samples',
    '_make_stationary_external_reference',
    '_latch_external_reference_fault',
    'get_reference_trajectory',
}


class _Logger:
    def warn(self, *args, **kwargs):
        pass

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
    obj.N = 20
    obj.skip_steps = 3
    obj.use_external_reference = True
    obj.traj = np.zeros((17, 100), dtype=float)
    obj.step_counter = 0
    obj.external_reference_timeout_s = 0.5
    obj.external_traj = None
    obj.last_external_reference_time = None
    obj.external_reference_state = 'WAITING'
    obj.external_reference_age_s = float('nan')
    obj.external_reference_fault_latched = False
    obj.external_reference_fault_reason = ''
    obj.external_reference_fault_position = None
    obj.external_reference_fault_latch_count = 0
    obj.current_pose = np.array(
        [1.0, 2.0, 3.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        dtype=float,
    )
    obj.takeoff_requested = False
    obj.xy_integral_error = np.zeros(2, dtype=float)
    obj.xy_integral_last_reference = None
    obj.xy_integral_last_tracking_error = np.zeros(2, dtype=float)
    obj.xy_integral_saturated = np.zeros(2, dtype=bool)
    obj.first_solve = False
    obj._test_now_s = time.time()
    obj._node_now_s = lambda: obj._test_now_s
    obj.get_logger = lambda: _Logger()
    return obj


def test_required_sample_count_is_exactly_61():
    controller = _controller_stub()
    assert controller._required_external_reference_samples() == 61


def test_missing_reference_before_takeoff_uses_nonlatched_ground_hold():
    controller = _controller_stub()
    traj, ref_step, external = controller.get_reference_trajectory()
    assert external is True
    assert ref_step == 0
    assert traj.shape == (17, 61)
    assert np.allclose(traj[0:3, :], controller.current_pose[0:3].reshape(3, 1))
    assert np.allclose(traj[3, :], 1.0)
    assert np.allclose(traj[4:17, :], 0.0)
    assert controller.external_reference_state == 'WAITING_GROUND_HOLD'
    assert controller.external_reference_fault_latched is False


def test_missing_reference_after_takeoff_latches_controlled_hold():
    controller = _controller_stub()
    controller.takeoff_requested = True
    traj, ref_step, external = controller.get_reference_trajectory()
    assert external is True
    assert ref_step == 0
    assert traj.shape == (17, 61)
    assert controller.external_reference_fault_latched is True
    assert controller.external_reference_state == 'FAULT_LATCHED_HOLD'
    assert controller.external_reference_fault_latch_count == 1
    assert np.allclose(controller.external_reference_fault_position, [1.0, 2.0, 3.0])
    assert controller.first_solve is True

    # A later valid planner message must not automatically resume motion.
    controller.external_traj = np.ones((17, 61), dtype=float)
    controller.last_external_reference_time = controller._node_now_s()
    resumed, _, _ = controller.get_reference_trajectory()
    assert controller.external_reference_state == 'FAULT_LATCHED_HOLD'
    assert np.allclose(resumed[0:3, 0], [1.0, 2.0, 3.0])
    assert np.allclose(resumed[7:13, :], 0.0)


def test_exact_61_sample_valid_reference_is_accepted_by_selector():
    controller = _controller_stub()
    controller.external_traj = np.zeros((17, 61), dtype=float)
    controller.external_traj[3, :] = 1.0
    controller.last_external_reference_time = controller._node_now_s()
    selected, ref_step, external = controller.get_reference_trajectory()
    assert external is True
    assert ref_step == 0
    assert selected.shape[1] == 61
    assert controller.external_reference_state == 'VALID'
    assert controller.external_reference_fault_latched is False
