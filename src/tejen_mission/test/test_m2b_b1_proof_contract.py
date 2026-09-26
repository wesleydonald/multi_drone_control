"""Tests-first contracts for simulation-feasible, IRL-retunable M2B proof motion."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest

from tejen_mission.m2b_attachment_mission import (
    M2BConfig,
    attached_hold_direction_plate,
    proof_direction_plate,
)


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "tejen_mission"
PLANNER = PKG / "tejen_mission" / "online_join_planner.py"
LAUNCH = PKG / "launch" / "m2b_b1_single_attachment.launch.py"


def test_sim_default_proof_is_radially_outward_and_hold_keeps_fifteen_degree_target():
    cfg = M2BConfig()
    np.testing.assert_allclose(proof_direction_plate(cfg), [1.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(
        attached_hold_direction_plate(cfg),
        [math.sin(math.radians(15.0)), 0.0, math.cos(math.radians(15.0))],
        atol=1e-12,
    )


def test_proof_direction_is_parameterized_normalized_and_independent_of_hold_angle():
    cfg = M2BConfig(
        proof_direction_plate_xyz=(2.0, -2.0, 0.0),
        attached_hold_angle_deg=20.0,
    )
    np.testing.assert_allclose(
        proof_direction_plate(cfg),
        np.array([1.0, -1.0, 0.0]) / math.sqrt(2.0),
        atol=1e-12,
    )
    np.testing.assert_allclose(
        attached_hold_direction_plate(cfg),
        [math.sin(math.radians(20.0)), 0.0, math.cos(math.radians(20.0))],
        atol=1e-12,
    )


def test_proof_direction_rejects_zero_or_nonfinite_vectors():
    with pytest.raises(ValueError):
        M2BConfig(proof_direction_plate_xyz=(0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        M2BConfig(proof_direction_plate_xyz=(1.0, float("nan"), 0.0))


def test_radial_25mm_proof_is_compatible_with_current_fixed_length_tether_geometry():
    # For the current 0.475 m anchor-to-contact distance, a 25 mm radial pull
    # requires only a sub-millimetre downward arc correction.  This documents
    # why the simulation default is radial rather than the old mostly-upward
    # 15-degree proof command.
    length = 0.475
    radial = 0.025
    vertical_drop = length - math.sqrt(length * length - radial * radial)
    assert vertical_drop < 0.001
    assert radial >= 0.012


def test_launch_uses_one_shared_set_of_proof_launch_arguments_for_planner_and_detector():
    text = LAUNCH.read_text(encoding="utf-8")
    for name in (
        "proof_direction_radial",
        "proof_direction_tangential",
        "proof_direction_normal",
        "proof_command_m",
        "proof_command_speed_mps",
        "proof_detector_speed_limit_mps",
        "proof_min_excitation_m",
        "attached_hold_angle_deg",
    ):
        assert f'LaunchConfiguration("{name}")' in text
        assert f'DeclareLaunchArgument("{name}"' in text

    # Same launch-level direction must feed both the software detector and
    # mission command generator, avoiding two hidden proof definitions.
    assert '"proof_direction_radial": ParameterValue(proof_direction_radial, value_type=float)' in text
    assert '"m2b_proof_direction_radial": ParameterValue(proof_direction_radial, value_type=float)' in text
    assert '"proof_direction_normal": ParameterValue(proof_direction_normal, value_type=float)' in text
    assert '"m2b_proof_direction_normal": ParameterValue(proof_direction_normal, value_type=float)' in text
    assert '"proof_speed_mps": ParameterValue(proof_detector_speed_limit_mps, value_type=float)' in text
    assert '"m2b_proof_speed_mps": ParameterValue(proof_command_speed_mps, value_type=float)' in text


def test_planner_exposes_all_proof_motion_and_acceptance_parameters():
    text = PLANNER.read_text(encoding="utf-8")
    for parameter in (
        "m2b_proof_direction_radial",
        "m2b_proof_direction_tangential",
        "m2b_proof_direction_normal",
        "m2b_proof_command_m",
        "m2b_proof_speed_mps",
        "m2b_proof_min_measured_excitation_m",
        "m2b_attached_hold_angle_deg",
    ):
        assert parameter in text


def test_b1_telemetry_records_exact_proof_and_hold_configuration_for_replay_and_irl_translation():
    telemetry = PKG / "tejen_mission" / "m2b_b1_telemetry.py"
    text = telemetry.read_text(encoding="utf-8")
    for key in (
        '"proof_direction_plate"',
        '"proof_command_m"',
        '"proof_command_speed_mps"',
        '"proof_detector_speed_limit_mps"',
        '"proof_min_excitation_m"',
        '"attached_hold_angle_deg"',
    ):
        assert key in text

    launch = LAUNCH.read_text(encoding="utf-8")
    assert '"proof_command_speed_mps": ParameterValue(proof_command_speed_mps, value_type=float)' in launch
    assert '"proof_detector_speed_limit_mps": ParameterValue(proof_detector_speed_limit_mps, value_type=float)' in launch
    assert '"attached_hold_angle_deg": ParameterValue(attached_hold_angle_deg, value_type=float)' in launch
