"""ROS-independent regressions for the existing guarded kT feedback policy.

M2B enables this already-existing controller path.  The tests execute the method
AST directly so the policy remains checkable in lightweight CI without rclpy.
"""

import ast
import math
from pathlib import Path
import time
import types

import numpy as np
import pytest


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "tejen_mpc"
    / "main.py"
)


def _feedback_method():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "Controller"
    )
    method = next(
        node for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name == "apply_thrust_ratio_feedback"
    )
    namespace = {"math": math, "np": np, "time": time}
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])),
            str(SOURCE),
            "exec",
        ),
        namespace,
    )
    return namespace["apply_thrust_ratio_feedback"]


class _Logger:
    def info(self, *args, **kwargs):
        pass


class _Dummy:
    def __init__(self):
        self.enable_thrust_ratio_feedback = True
        self.thrust_ratio_estimator_backend = "full_model_kt_ukf"
        self.est_params = np.array([44.0], dtype=float)
        self.initial_mpc_thrust_ratio = 44.0
        self.thrust_ratio_feedback_min_updates = 15
        self.thrust_ratio_feedback_max_std = 1.5
        self.thrust_ratio_feedback_rate_per_s = 0.5
        self.thrust_ratio_feedback_max_fractional_change = 0.25
        self.thrust_ratio_feedback_deadband = 0.20
        self.thrust_ratio_feedback_last_time = None
        self.thrust_ratio_estimator_period_s = 0.10
        self.thrust_ratio_full_model_max_dt_s = 0.25
        self.thrust_ratio_ukf_min = 12.0
        self.thrust_ratio_ukf_max = 60.0
        self.thrust_ratio_feedback_applied = False
        self.thrust_ratio_feedback_target = 44.0
        self.thrust_ratio_feedback_status = "waiting_for_estimator"
        self.current_payload_mpc_mode = "FREE_SWING"

    def _node_now_s(self):
        return time.monotonic()

    def get_logger(self):
        return _Logger()


def _result(*, filtered=43.0, variance=0.25, updates=20, initialized=True, updated=True, status="updated"):
    return {
        "initialized": initialized,
        "updated": updated,
        "status": status,
        "update_count": updates,
        "filtered_thrust_ratio": filtered,
        "thrust_ratio_variance": variance,
    }


def test_feedback_does_not_move_mpc_before_update_count_gate():
    dummy = _Dummy()
    _feedback_method()(dummy, _result(updates=14))
    assert dummy.est_params[0] == pytest.approx(44.0)
    assert dummy.thrust_ratio_feedback_applied is False
    assert dummy.thrust_ratio_feedback_status == "waiting_for_min_updates"


def test_feedback_rejects_estimate_with_excessive_uncertainty():
    dummy = _Dummy()
    _feedback_method()(dummy, _result(variance=1.51**2))
    assert dummy.est_params[0] == pytest.approx(44.0)
    assert dummy.thrust_ratio_feedback_applied is False
    assert dummy.thrust_ratio_feedback_status == "uncertainty_too_high"


def test_valid_feedback_is_slew_limited_before_updating_mpc_model():
    dummy = _Dummy()
    _feedback_method()(dummy, _result(filtered=43.0, variance=0.46**2, updates=20))
    # First eligible 10 Hz update: 0.5 kT/s * 0.1 s = 0.05 maximum change.
    assert dummy.est_params[0] == pytest.approx(43.95)
    assert dummy.thrust_ratio_feedback_target == pytest.approx(43.0)
    assert dummy.thrust_ratio_feedback_applied is True
    assert dummy.thrust_ratio_feedback_status == "applied"


def test_feedback_inside_deadband_leaves_model_unchanged():
    dummy = _Dummy()
    _feedback_method()(dummy, _result(filtered=43.85, variance=0.25, updates=20))
    assert dummy.est_params[0] == pytest.approx(44.0)
    assert dummy.thrust_ratio_feedback_applied is False
    assert dummy.thrust_ratio_feedback_status == "within_deadband"


def test_feedback_is_frozen_when_m2_is_physically_constrained_for_proof_or_hold():
    method = _feedback_method()
    for mode in ("ATTACH_PROOF", "ATTACHED_HOLD"):
        dummy = _Dummy()
        dummy.current_payload_mpc_mode = mode
        method(dummy, _result(filtered=40.0, variance=0.25, updates=50))
        assert dummy.est_params[0] == pytest.approx(44.0)
        assert dummy.thrust_ratio_feedback_applied is False
        assert dummy.thrust_ratio_feedback_status == "suspended_for_constrained_mode"
