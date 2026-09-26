"""Tests-first behavioural contract for M2B single-drone attachment primitives.

These tests intentionally exercise ROS-independent policy helpers.  The ROS mission
executive remains ``online_join_planner``; M2B adds mission-specific policy without
creating a second planner architecture.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from tejen_mission.mission_definitions import mission_for_mode
from tejen_mission.mission_types import MissionPhase
from tejen_mission.m2b_attachment_mission import (
    BootstrapStatus,
    M2BConfig,
    M2BBootstrapGate,
    M2BDescentProgress,
    M2BRetryTracker,
    body_reference_from_contact,
    contact_capture_position,
    m2b_mpc_mode_for_phase,
    proof_command_distance,
    proof_success,
    should_enable_magnet,
    yaw_quaternion_xyzw,
)


def test_m2b_is_a_normal_selectable_mission_in_shared_framework():
    mission = mission_for_mode("m2b")
    assert mission.initial_phase == MissionPhase.M2_BOOTSTRAP_DETACH
    required = {
        MissionPhase.M2_BOOTSTRAP_DETACH,
        MissionPhase.M2_BOOTSTRAP_SETTLE,
        MissionPhase.M2_WAIT_FOR_ARM,
        MissionPhase.M2_VERTICAL_TAKEOFF,
        MissionPhase.M2_TAKEOFF_HOVER,
        MissionPhase.M2_SETTLE_CAPTURE,
        MissionPhase.M2_ATTACH_APPROACH_HIGH,
        MissionPhase.M2_ATTACH_APPROACH_LOW,
        MissionPhase.M2_ATTACH_CAPTURE_WAIT,
        MissionPhase.M2_ATTACH_PROOF,
        MissionPhase.M2_ATTACHED_HOLD,
        MissionPhase.M2_DETACHED_RETREAT,
        MissionPhase.M2_LANDING_STAGE,
        MissionPhase.M2_FAULT,
        MissionPhase.LANDING,
        MissionPhase.LANDED_DISARMED,
    }
    assert required.issubset(set(mission.phases))


def test_bootstrap_cannot_arm_until_fresh_detached_dwell_and_settle():
    cfg = M2BConfig(bootstrap_detach_dwell_s=0.15, bootstrap_settle_s=0.50)
    gate = M2BBootstrapGate(cfg, start_time_s=0.0)

    out = gate.step(0.00, joint_truth_fresh=True, joint_detached=False)
    assert out.status == BootstrapStatus.DETACHING
    assert out.request_raw_detach is True
    assert out.arm_permitted is False

    out = gate.step(0.10, joint_truth_fresh=True, joint_detached=True)
    assert out.arm_permitted is False
    out = gate.step(0.24, joint_truth_fresh=True, joint_detached=True)
    assert out.status == BootstrapStatus.DETACHING
    out = gate.step(0.26, joint_truth_fresh=True, joint_detached=True)
    assert out.status == BootstrapStatus.SETTLING
    assert out.arm_permitted is False
    out = gate.step(0.74, joint_truth_fresh=True, joint_detached=True)
    assert out.arm_permitted is False
    out = gate.step(0.77, joint_truth_fresh=True, joint_detached=True)
    assert out.status == BootstrapStatus.ARMABLE
    assert out.request_raw_detach is False
    assert out.arm_permitted is True


def test_bootstrap_stale_or_reattached_truth_resets_progress():
    cfg = M2BConfig(bootstrap_detach_dwell_s=0.10, bootstrap_settle_s=0.20)
    gate = M2BBootstrapGate(cfg, start_time_s=0.0)
    gate.step(0.00, joint_truth_fresh=True, joint_detached=True)
    gate.step(0.11, joint_truth_fresh=True, joint_detached=True)
    assert gate.step(0.20, joint_truth_fresh=False, joint_detached=True).status == BootstrapStatus.DETACHING
    assert gate.step(0.31, joint_truth_fresh=True, joint_detached=True).arm_permitted is False
    assert gate.step(0.32, joint_truth_fresh=True, joint_detached=False).request_raw_detach is True


def test_bootstrap_timeout_is_a_disarmed_fault():
    cfg = M2BConfig(bootstrap_timeout_s=1.0)
    gate = M2BBootstrapGate(cfg, start_time_s=10.0)
    out = gate.step(11.01, joint_truth_fresh=False, joint_detached=False)
    assert out.status == BootstrapStatus.FAULT
    assert out.arm_permitted is False
    assert out.request_raw_detach is False


def test_low_band_is_strictly_tighter_than_high_band():
    cfg = M2BConfig(high_xy_tolerance_m=0.05, low_xy_tolerance_m=0.025)
    assert cfg.low_xy_tolerance_m < cfg.high_xy_tolerance_m
    with pytest.raises(ValueError):
        M2BConfig(high_xy_tolerance_m=0.02, low_xy_tolerance_m=0.02)


def test_high_band_xy_error_freezes_z_and_then_resumes():
    cfg = M2BConfig(
        high_low_split_m=0.08,
        high_xy_tolerance_m=0.05,
        low_xy_tolerance_m=0.02,
        descent_speed_mps=0.04,
        alignment_pause_timeout_s=2.0,
    )
    progress = M2BDescentProgress(cfg, initial_contact_height_m=0.30)

    paused = progress.step(now_s=0.0, measured_xy_error_m=0.07, dt_s=0.1)
    assert paused.paused_for_xy is True
    assert paused.target_contact_height_m == pytest.approx(0.30)
    assert paused.failed is False

    moving = progress.step(now_s=0.1, measured_xy_error_m=0.03, dt_s=0.1)
    assert moving.paused_for_xy is False
    assert moving.target_contact_height_m == pytest.approx(0.296)


def test_low_band_uses_tighter_gate_and_gross_or_persistent_error_fails():
    cfg = M2BConfig(
        high_low_split_m=0.08,
        high_xy_tolerance_m=0.05,
        low_xy_tolerance_m=0.02,
        gross_xy_abort_m=0.10,
        alignment_pause_timeout_s=0.5,
    )
    progress = M2BDescentProgress(cfg, initial_contact_height_m=0.07)
    out = progress.step(now_s=0.0, measured_xy_error_m=0.03, dt_s=0.1)
    assert out.band == "LOW"
    assert out.paused_for_xy is True
    assert progress.step(now_s=0.51, measured_xy_error_m=0.03, dt_s=0.1).failed is True

    progress.reset(0.20)
    assert progress.step(now_s=1.0, measured_xy_error_m=0.11, dt_s=0.1).failed is True


def test_magnet_cannot_enable_too_early_or_on_stale_inputs():
    cfg = M2BConfig(magnet_enable_height_m=0.04, low_xy_tolerance_m=0.02)
    assert should_enable_magnet(cfg, contact_height_m=0.05, xy_error_m=0.0, detector_fresh=True, relative_speed_mps=0.0) is False
    assert should_enable_magnet(cfg, contact_height_m=0.03, xy_error_m=0.01, detector_fresh=False, relative_speed_mps=0.0) is False
    assert should_enable_magnet(cfg, contact_height_m=0.03, xy_error_m=0.03, detector_fresh=True, relative_speed_mps=0.0) is False
    assert should_enable_magnet(cfg, contact_height_m=0.03, xy_error_m=0.01, detector_fresh=True, relative_speed_mps=0.05) is True


def test_commanded_proof_is_not_success_without_measured_evidence():
    cfg = M2BConfig(proof_command_m=0.025, proof_speed_mps=0.03, proof_min_measured_excitation_m=0.012)
    assert proof_command_distance(cfg, elapsed_s=5.0) == pytest.approx(0.025)
    assert proof_success(cfg, detector_confirmed=False, detector_fresh=True, physical_latched=True, measured_excitation_m=0.025) is False
    assert proof_success(cfg, detector_confirmed=True, detector_fresh=True, physical_latched=True, measured_excitation_m=0.011) is False
    assert proof_success(cfg, detector_confirmed=True, detector_fresh=True, physical_latched=True, measured_excitation_m=0.012) is True


def test_retry_stays_on_same_plate_and_third_failure_is_terminal():
    tracker = M2BRetryTracker(assigned_plate_id=7, max_attempts=3)
    assert tracker.assigned_plate_id == 7
    first = tracker.record_failure()
    second = tracker.record_failure()
    third = tracker.record_failure()
    assert first.retry_allowed and first.assigned_plate_id == 7 and first.attempt_number == 2
    assert second.retry_allowed and second.assigned_plate_id == 7 and second.attempt_number == 3
    assert third.retry_allowed is False and third.terminal_failure is True
    assert tracker.assigned_plate_id == 7


def test_capture_height_is_contact_height_and_body_target_respects_anchor_offset():
    plate = np.array([0.25, -0.2, 0.10])
    contact = contact_capture_position(plate, capture_height_m=0.30)
    np.testing.assert_allclose(contact, [0.25, -0.2, 0.40])

    direction = np.array([0.0, 0.0, 1.0])
    rotation_world_from_body = np.eye(3)
    anchor_body = np.array([0.02, -0.01, -0.05])
    body = body_reference_from_contact(
        contact_position_world=contact,
        direction_contact_to_vehicle_world=direction,
        tether_length_m=0.50,
        rotation_world_from_body=rotation_world_from_body,
        tether_anchor_body=anchor_body,
    )
    np.testing.assert_allclose(body, contact + [0.0, 0.0, 0.50] - anchor_body)


def test_initial_yaw_can_be_preserved_exactly_in_reference_quaternion():
    yaw = math.radians(73.0)
    q = yaw_quaternion_xyzw(yaw)
    assert q[0] == pytest.approx(0.0)
    assert q[1] == pytest.approx(0.0)
    assert q[2] == pytest.approx(math.sin(yaw / 2.0))
    assert q[3] == pytest.approx(math.cos(yaw / 2.0))


def test_phase_to_mpc_mode_contract():
    assert m2b_mpc_mode_for_phase(MissionPhase.M2_VERTICAL_TAKEOFF) == "VERTICAL_TAKEOFF"
    assert m2b_mpc_mode_for_phase(MissionPhase.M2_ATTACH_PROOF) == "ATTACH_PROOF"
    assert m2b_mpc_mode_for_phase(MissionPhase.M2_ATTACHED_HOLD) == "ATTACHED_HOLD"
    assert m2b_mpc_mode_for_phase(MissionPhase.M2_DETACHED_RETREAT) == "DETACHED_RETREAT"
    assert m2b_mpc_mode_for_phase(MissionPhase.M2_ATTACH_APPROACH_HIGH) == "ATTACH_APPROACH"
    assert m2b_mpc_mode_for_phase(MissionPhase.M2_TAKEOFF_HOVER) == "FREE_SWING"


def test_m2b_vertical_takeoff_uses_smooth_acceleration_bounded_reference_only():
    source = (
        Path(__file__).resolve().parents[1]
        / "tejen_mission"
        / "online_join_planner.py"
    ).read_text(encoding="utf-8")
    assert 'takeoff_reference_acceleration_limit_mps2' in source
    assert 'smooth_rate_controlled_manoeuvre(' in source
    # The commissioned generic helper is intentionally retained for attachment,
    # proof, retreat and landing phases; this increment changes takeoff onset only.
    assert source.count('rate_controlled_manoeuvre(') >= 3
