"""
Regression tests for the ATTACH control-authority handover rule.

WHAT THIS PINS, AND WHY IT EXISTS
---------------------------------
Every Gazebo ring-attach run diverged, and the cause was not the control law or the weld
geometry: `elrs_mux` handed authority to our tracker on the first /magnet/object_attached
while that tracker still had no reference and was therefore publishing armed-idle
(channel_2 = -1.0, motors off). Measured across R0031-R0046, 12 welds in 12 runs: drone 3
throttle 0.36 -> 0.000 within 0.03 s of the weld, ~0.2 s of dead motors, body tilt 6 ->
89 deg, ENVELOPE FAULT, fleet abort. The tilt was a CONSEQUENCE -- at zero throttle the
betaflight mixer clips its rate offsets against zero and cannot hold attitude either.

These tests cover the decision only. Two other pieces of the fix are elsewhere and are
NOT covered here: dissipative_node._publish_pending_attach_refs (which keeps the tracker
warm so `require_live` passes immediately), and controller_mpc's airborne no-reference
branch. Only the SIL bench and Gazebo exercise those together.
"""
import pytest

from drone_magnet.handover_policy import HandoverPolicy, IDLE_THROTTLE

FLYING = 0.35 * 2 - 1        # a hovering tracker's channel_2 (~-0.30)


def test_no_handover_before_the_weld():
    p = HandoverPolicy()
    assert p.tracker_command(True, FLYING, 0.0) is False
    assert p.attached(0.0) is False


def test_the_documented_failure_is_refused():
    """Welded, but the tracker is publishing armed-idle: authority must NOT transfer."""
    p = HandoverPolicy()
    p.tracker_command(True, -1.0, 0.0)          # armed-idle, throttle zero
    assert p.weld(True, 0.1) is False
    assert p.tracker_command(True, -1.0, 0.2) is False


def test_handover_on_the_first_flying_command_after_the_weld():
    p = HandoverPolicy()
    assert p.weld(True, 0.0) is False           # tracker silent so far
    assert p.tracker_command(True, FLYING, 0.02) is True


def test_a_warm_tracker_hands_over_on_the_weld_itself():
    """With the pre-weld hold reference published, there is no gap at all."""
    p = HandoverPolicy()
    p.tracker_command(True, FLYING, 0.0)
    assert p.weld(True, 0.01) is True


def test_liveness_expires():
    p = HandoverPolicy(live_timeout_s=0.5)
    p.tracker_command(True, FLYING, 0.0)
    assert p.weld(True, 0.4) is True
    assert p.attached(0.6) is False             # stream went quiet


def test_disarmed_commands_are_not_live():
    p = HandoverPolicy()
    p.tracker_command(False, FLYING, 0.0)
    assert p.weld(True, 0.1) is False


def test_idle_threshold_sits_just_above_full_down():
    """-1.0 must read as idle; the hover command must not."""
    assert -1.0 <= IDLE_THROTTLE < FLYING


def test_weld_latches_by_default():
    p = HandoverPolicy()
    p.tracker_command(True, FLYING, 0.0)
    p.weld(True, 0.1)
    assert p.weld(False, 0.2) is True           # a dropped signal cannot hand back


def test_unlatched_weld_releases():
    p = HandoverPolicy(latch=False)
    p.tracker_command(True, FLYING, 0.0)
    p.weld(True, 0.1)
    assert p.weld(False, 0.2) is False


def test_require_live_off_restores_the_old_behaviour():
    p = HandoverPolicy(require_live=False)
    assert p.weld(True, 0.0) is True            # switches onto a silent stream


@pytest.mark.parametrize('throttle', [-1.0, -0.999, IDLE_THROTTLE])
def test_idle_throttles_never_count_as_flying(throttle):
    p = HandoverPolicy()
    p.tracker_command(True, throttle, 0.0)
    assert p.tracker_live(0.0) is False
