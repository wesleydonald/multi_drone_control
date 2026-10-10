"""Focused regressions for IRL-only measured-tip alignment and bounded retries."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from tejen_mission.m2b_attachment_mission import M2BIRLTipAlignment, M2BRetryTracker


def _step(alignment, now, error, *, normal=0.20, fresh=True, magnet_on=False, phase="capture"):
    return alignment.step(
        now_s=now,
        error_world_xy=np.array([error, 0.0]),
        xy_error_m=abs(error),
        normal_error_m=normal,
        relative_speed_mps=0.01,
        fresh=fresh,
        magnet_on=magnet_on,
        phase=phase,
    )


def test_trim_converges_for_logged_bias_with_rate_and_offset_bounds():
    alignment = M2BIRLTipAlignment()
    previous = np.zeros(2)
    for n in range(240):
        decision = _step(alignment, n / 30.0, 0.054 - alignment.offset_xy[0])
        assert np.linalg.norm(decision.offset_xy - previous) <= 0.025 / 30.0 + 1e-9
        assert np.linalg.norm(decision.offset_xy) <= 0.08 + 1e-9
        previous = decision.offset_xy.copy()
    assert abs(0.054 - alignment.offset_xy[0]) < 0.005


def test_trim_freezes_on_stale_near_plate_or_magnet_on_and_survives_retry():
    alignment = M2BIRLTipAlignment()
    _step(alignment, 0.0, 0.06)
    _step(alignment, 1.0 / 30, 0.06)
    saved = alignment.offset_xy.copy()
    _step(alignment, 2.0 / 30, 0.06, fresh=False)
    _step(alignment, 3.0 / 30, 0.06, normal=0.05, phase="high")
    _step(alignment, 4.0 / 30, 0.06, magnet_on=True, phase="high")
    _step(alignment, 5.0 / 30, 0.06, phase="low")
    np.testing.assert_allclose(alignment.offset_xy, saved)
    alignment.start_retry()
    np.testing.assert_allclose(alignment.offset_xy, saved)
    _step(alignment, 6.0 / 30, 0.06, phase="capture")
    assert alignment.offset_xy[0] > saved[0]
    alignment.reset_assignment()
    np.testing.assert_allclose(alignment.offset_xy, [0.0, 0.0])


def test_capture_and_pre_low_require_fresh_tip_dwell_at_safe_clearance():
    alignment = M2BIRLTipAlignment()
    for n in range(24):
        decision = _step(alignment, n / 30.0, 0.015)
    assert decision.capture_ready
    decision = _step(alignment, 24 / 30.0, 0.04, normal=0.10, phase="high")
    assert not decision.pre_low_ready
    for n in range(25, 37):
        decision = _step(alignment, n / 30.0, 0.02, normal=0.10, phase="high")
    assert decision.pre_low_ready
    decision = _step(alignment, 37 / 30.0, 0.02, normal=0.06, phase="high")
    assert not decision.pre_low_ready
    assert decision.raise_to_safe_clearance


def test_same_plate_heartbeat_cannot_reset_retry_count():
    from tejen_mission.m2b_attachment_mission import retry_tracker_for_assignment

    tracker = M2BRetryTracker(assigned_plate_id=6, max_attempts=3)
    tracker.record_failure()
    assert retry_tracker_for_assignment(tracker, 6, max_attempts=3) is tracker
    assert tracker.record_failure().attempt_number == 3
    assert tracker.record_failure().terminal_failure
    assert retry_tracker_for_assignment(tracker, 7, max_attempts=3).failures == 0


def test_planner_uses_tip_trim_only_for_irl_local_capture_and_approach():
    planner = (Path(__file__).resolve().parents[1] / "tejen_mission" / "online_join_planner.py").read_text()
    assert 'self.declare_parameter("m2b_irl_tip_alignment_enabled", False)' in planner
    assert "self.m2b_irl_tip_alignment.offset_xy" in planner
    assert "retry_tracker_for_assignment(" in planner



def test_irl_high_uses_existing_five_centimetre_gate_until_staging():
    from tejen_mission.m2b_attachment_mission import M2BConfig, M2BDescentProgress

    planner = (Path(__file__).resolve().parents[1] / "tejen_mission" / "online_join_planner.py").read_text()
    high = planner.split(
        "if self.m2b_irl_tip_alignment_enabled and self.phase == MissionPhase.M2_ATTACH_APPROACH_HIGH:", 1
    )[1].split("decision = self.m2b_descent_progress.step(", 1)[0]
    assert 'self.m2b_irl_tip_error_m > self.m2b_irl_tip_alignment.config.low_xy_m' not in high
    assert 'tip.reason == "relative_speed"' not in high
    assert "tip.pre_low_active" in high
    assert "self.m2b_config.high_low_split_m + self.m2b_config.descent_speed_mps * self.dt" in high
    assert "self.m2b_config.gross_xy_abort_m" in high

    progress = M2BDescentProgress(M2BConfig(), initial_contact_height_m=0.20)
    decision = progress.step(now_s=0.0, measured_xy_error_m=0.04, dt_s=0.1)
    assert not decision.paused_for_xy
    assert decision.target_contact_height_m < 0.20



def test_high_alignment_dwell_can_finish_before_reaching_staging_clearance():
    alignment = M2BIRLTipAlignment()
    for n in range(12):
        decision = _step(alignment, n / 30.0, 0.015, normal=0.20, phase="high")
        assert not decision.pre_low_ready
    decision = _step(alignment, 12 / 30.0, 0.015, normal=0.10, phase="high")
    assert decision.pre_low_ready

    # A later loss of evidence must cancel the earned dwell.
    _step(alignment, 13 / 30.0, 0.015, normal=0.10, fresh=False, phase="high")
    decision = _step(alignment, 14 / 30.0, 0.015, normal=0.10, phase="high")
    assert not decision.pre_low_ready


def test_capture_body_settle_does_not_wait_for_narrow_tip_dwell():
    planner = (Path(__file__).resolve().parents[1] / "tejen_mission" / "online_join_planner.py").read_text()
    capture = planner.split("if self.phase == MissionPhase.M2_SETTLE_CAPTURE:", 1)[1].split(
        "if self.phase in {MissionPhase.M2_ATTACH_APPROACH_HIGH", 1
    )[0]
    assert "if body_settled:" in capture
    assert "tip.capture_ready" not in capture
    assert "IRL tip alignment timeout at capture" not in capture
