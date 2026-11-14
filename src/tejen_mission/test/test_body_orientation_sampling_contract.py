"""Adapter ordering contracts; native tests cover quaternion/geometry maths."""
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
BACKEND = ROOT / 'src/tejen_dynamic_planner/src/transfer_backend_node.cpp'


def method(start, end):
    return BACKEND.read_text().split(start, 1)[1].split(end, 1)[0]


def test_state_receipt_keeps_fresh_normalized_orientation_without_mutating_collision_world():
    state = method('void stateCallback(', 'void latchBodyOrientation()')
    assert 'orientation.coeffs().allFinite()' in state
    assert state.index('kinematics_.update') < state.index('latest_body_orientation_ = orientation.normalized();')
    assert 'last_state_receive_ros_s_ = now_s;' in state
    assert 'setEgoBodyOrientation' not in state
    assert 'latchBodyOrientation' not in state


@pytest.mark.parametrize('start,end', [
    ('bool tryActivate()', 'void pollPlannerFuture()'),
    ('void authorityCallback(', 'void disableBackend'),
    ('void replanTimer()', 'void referenceTimer()'),
])
def test_new_transactions_latch_only_after_fresh_state_and_no_pending_worker(start, end):
    body = method(start, end)
    assert body.index('planner_future_.valid()') < body.index('latchBodyOrientation();')
    assert body.index('currentState(now_s, &measured, &age_s)') < body.index('latchBodyOrientation();')
    if start == 'void replanTimer()':
        assert body.index('latchBodyOrientation();') < body.index('auto world = world_;')
        assert 'refreshCooperativeWorld(now_s)' in body


def test_installation_does_not_refresh_rotation_and_fallback_cancels_before_sampling():
    assert 'latchBodyOrientation' not in method('void pollPlannerFuture()', 'void replanTimer()')
    fallback = method('ExecutionFallbackOutcome activateExecutionFallback(', 'bool executionTrackingCertificateValid()')
    assert fallback.index('++epoch_;') < fallback.index('latchBodyOrientation();')
    assert fallback.index('latchBodyOrientation();') < fallback.index('world_->snapshot(now_s)')


def test_latch_is_measured_mode_only_and_uses_stored_actual_quaternion():
    source = BACKEND.read_text()
    helper = method('void latchBodyOrientation()', 'void targetCallback(')
    assert 'body_orientation_mode_ == "measured_snapshot"' in helper
    assert 'world_->setEgoBodyOrientation(latest_body_orientation_);' in helper
    assert source.count('world_->setEgoBodyOrientation(') == 1
