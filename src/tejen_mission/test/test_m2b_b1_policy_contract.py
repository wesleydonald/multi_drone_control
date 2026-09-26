"""Tests-first pure behavioural contracts for M2B B1 local attachment."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from tejen_mission.m2b_attachment_mission import (
    M2BConfig,
    M2BSettleGate,
    evaluate_takeoff_magnet_clearance,
    should_enable_magnet,
)


def test_settle_gate_rejects_observed_b0_post_takeoff_transient_then_accepts_quiet_hover():
    cfg = M2BConfig(
        settle_position_tolerance_m=0.05,
        settle_speed_mps=0.03,
        settle_swing_deg=3.0,
        settle_dwell_s=0.75,
    )
    gate = M2BSettleGate(cfg)

    # Runtime evidence from the commissioned B0 first-hover transient was roughly
    # 0.24 m/s and 24 deg: this must not be treated as settled.
    out = gate.step(
        now_s=0.0,
        inputs_fresh=True,
        position_error_m=0.01,
        speed_mps=0.24,
        swing_magnitude_rad=math.radians(24.0),
    )
    assert out.settled is False
    assert out.dwell_s == pytest.approx(0.0)

    # By 5-10 s after hover, B0 evidence was <=~0.017 m/s and <=~1.3 deg.
    assert gate.step(
        now_s=5.0,
        inputs_fresh=True,
        position_error_m=0.02,
        speed_mps=0.017,
        swing_magnitude_rad=math.radians(1.3),
    ).settled is False
    out = gate.step(
        now_s=5.76,
        inputs_fresh=True,
        position_error_m=0.02,
        speed_mps=0.01,
        swing_magnitude_rad=math.radians(1.0),
    )
    assert out.settled is True
    assert out.dwell_s >= 0.75


def test_settle_gate_requires_continuous_fresh_evidence_and_position_convergence():
    cfg = M2BConfig(settle_dwell_s=0.5)
    gate = M2BSettleGate(cfg)
    gate.step(
        now_s=1.0,
        inputs_fresh=True,
        position_error_m=0.01,
        speed_mps=0.0,
        swing_magnitude_rad=0.0,
    )
    stale = gate.step(
        now_s=1.3,
        inputs_fresh=False,
        position_error_m=0.01,
        speed_mps=0.0,
        swing_magnitude_rad=0.0,
    )
    assert stale.settled is False
    assert stale.dwell_s == pytest.approx(0.0)

    # A fresh sample after stale data starts a new dwell, it cannot inherit it.
    assert gate.step(
        now_s=1.6,
        inputs_fresh=True,
        position_error_m=0.01,
        speed_mps=0.0,
        swing_magnitude_rad=0.0,
    ).settled is False

    gate.reset()
    too_far = gate.step(
        now_s=2.0,
        inputs_fresh=True,
        position_error_m=cfg.settle_position_tolerance_m + 0.001,
        speed_mps=0.0,
        swing_magnitude_rad=0.0,
    )
    assert too_far.settled is False


def test_takeoff_clearance_requires_fresh_measured_magnet_rise_and_has_bounded_timeout():
    grounded = evaluate_takeoff_magnet_clearance(
        enabled=True,
        start_tip_z_m=0.038,
        current_tip_z_m=0.038,
        inputs_fresh=True,
        minimum_rise_m=0.03,
        hover_elapsed_s=0.0,
        timeout_s=15.0,
    )
    assert grounded.ready is False
    assert grounded.timed_out is False
    assert grounded.reason == "magnet has not cleared ground datum"

    stale = evaluate_takeoff_magnet_clearance(
        enabled=True,
        start_tip_z_m=0.038,
        current_tip_z_m=0.080,
        inputs_fresh=False,
        minimum_rise_m=0.03,
        hover_elapsed_s=15.0,
        timeout_s=15.0,
    )
    assert stale.ready is False
    assert stale.timed_out is True
    assert stale.reason == "magnet-tip evidence unavailable/stale"

    clear = evaluate_takeoff_magnet_clearance(
        enabled=True,
        start_tip_z_m=0.038,
        current_tip_z_m=0.068,
        inputs_fresh=True,
        minimum_rise_m=0.03,
        hover_elapsed_s=2.0,
        timeout_s=15.0,
    )
    assert clear.ready is True
    assert clear.timed_out is False

    simulation_default = evaluate_takeoff_magnet_clearance(
        enabled=False,
        start_tip_z_m=None,
        current_tip_z_m=None,
        inputs_fresh=False,
        minimum_rise_m=0.03,
        hover_elapsed_s=99.0,
        timeout_s=15.0,
    )
    assert simulation_default.ready is True
    assert simulation_default.timed_out is False


def test_planner_gates_post_takeoff_settle_on_magnet_clearance_and_bounded_timeout():
    source = (
        Path(__file__).resolve().parents[1]
        / "tejen_mission"
        / "online_join_planner.py"
    ).read_text(encoding="utf-8")
    command = source.split("def drone_command_callback", 1)[1].split(
        "def external_landing_callback", 1
    )[0]
    hover_gate = source.split("def m2b_takeoff_hover_settled", 1)[1].split(
        "def m2b_request_detector_reset", 1
    )[0]
    assert "self.m2b_takeoff_magnet_start_z = float(" in command
    assert "evaluate_takeoff_magnet_clearance(" in hover_gate
    assert "self.m2b_takeoff_settle_gate.reset()" in hover_gate
    assert "MissionPhase.M2_FAULT" in hover_gate
    assert source.count("self.m2b_takeoff_hover_settled(") == 2


def test_magnet_height_gate_is_symmetric_about_plate_plane():
    cfg = M2BConfig(magnet_enable_height_m=0.04, low_xy_tolerance_m=0.025)
    assert should_enable_magnet(
        cfg,
        contact_height_m=0.03,
        xy_error_m=0.01,
        detector_fresh=True,
        relative_speed_mps=0.03,
    )
    # Penetrating 20 cm below the plate must never look "close enough" merely
    # because its signed normal error is numerically <= +0.04.
    assert not should_enable_magnet(
        cfg,
        contact_height_m=-0.20,
        xy_error_m=0.01,
        detector_fresh=True,
        relative_speed_mps=0.03,
    )


def test_b1_config_rejects_nonsensical_settle_or_retry_values():
    with pytest.raises(ValueError):
        M2BConfig(settle_speed_mps=0.0)
    with pytest.raises(ValueError):
        M2BConfig(settle_swing_deg=0.0)
    with pytest.raises(ValueError):
        M2BConfig(settle_position_tolerance_m=0.0)
    with pytest.raises(ValueError):
        M2BConfig(attached_loss_grace_s=-0.1)


def test_capture_wait_is_entered_from_measured_contact_band_not_commanded_zero_height():
    from tejen_mission.m2b_attachment_mission import should_enter_capture_wait

    cfg = M2BConfig(capture_hold_normal_m=0.020)
    # Known vertical tracking bias must not force the commanded contact target to
    # cross the plate before the mission starts holding for physical capture.
    assert should_enter_capture_wait(cfg, contact_height_m=0.015, magnet_enabled=True)
    assert should_enter_capture_wait(cfg, contact_height_m=-0.015, magnet_enabled=True)
    assert not should_enter_capture_wait(cfg, contact_height_m=0.025, magnet_enabled=True)
    assert not should_enter_capture_wait(cfg, contact_height_m=0.010, magnet_enabled=False)


def test_approach_stale_evidence_is_bounded_and_fresh_data_resets_the_timer():
    from tejen_mission.m2b_attachment_mission import M2BEvidenceFreshnessGate

    cfg = M2BConfig(approach_evidence_loss_abort_s=0.75)
    gate = M2BEvidenceFreshnessGate(cfg)
    first = gate.step(now_s=1.0, fresh=False)
    assert first.failed is False
    assert first.bad_duration_s == pytest.approx(0.0)
    assert gate.step(now_s=1.74, fresh=False).failed is False
    assert gate.step(now_s=1.76, fresh=False).failed is True

    recovered = gate.step(now_s=2.0, fresh=True)
    assert recovered.failed is False
    assert recovered.bad_duration_s == pytest.approx(0.0)
    assert gate.step(now_s=2.1, fresh=False).failed is False
