"""Focused IRL-only bottom-well and plate-fixed attachment proof regressions."""

from pathlib import Path

import numpy as np
import yaml

from tejen_mission.attachment_detection import (
    AttachmentDetector,
    AttachmentObservation,
    MagnetContactCalibration,
)
from tejen_mission.cooperative_trajectory import RingNetGeometry
from tejen_mission.m2a_attachment_runtime import default_detector_config


CONFIG = Path(__file__).resolve().parents[1] / "config" / "m2_irl_two_drone.yaml"


def _observer(config):
    geometry = RingNetGeometry()
    detector = AttachmentDetector(
        geometry=geometry,
        assigned_plate_id=0,
        calibration=MagnetContactCalibration(np.zeros(3), np.eye(3)),
        config=config,
    )
    plate = geometry.plate_body_offset(0)
    return detector, plate


def _sample(detector, plate, t, *, magnet_delta=(0, 0, 0), drone_delta=(0, 0, 0), proof=False, magnet_time_s=None):
    return detector.update(AttachmentObservation(
        time_s=t,
        ring_position_world=np.zeros(3),
        rotation_world_from_ring=np.eye(3),
        ring_time_s=t,
        magnet_position_world=plate + np.asarray(magnet_delta, dtype=float),
        rotation_world_from_magnet=np.eye(3),
        magnet_time_s=t if magnet_time_s is None else magnet_time_s,
        drone_position_world=plate + np.array([0.0, 0.0, 0.6]) + np.asarray(drone_delta, dtype=float),
        drone_time_s=t,
        magnet_on=True,
        proof_requested=proof,
    ))


def test_irl_profile_keeps_search_swing_separate_from_slow_proof():
    from tejen_mission.m2_irl_launch_config import parse_gate_profiles

    profile = parse_gate_profiles(yaml.safe_load(CONFIG.read_text())).attachment_search
    assert not hasattr(profile, "search_timeout_s")
    assert profile.contact_overtravel_m == 0.010
    assert profile.candidate_xy_m == 0.025
    assert not hasattr(profile, "alignment_pause_timeout_s")
    assert profile.capture_normal_m == 0.015
    assert profile.candidate_normal_m == 0.015
    assert profile.proof_normal_m == 0.015
    assert profile.proof_relative_speed_mps == 0.030
    assert profile.proof_max_contact_drift_m == 0.040
    assert default_detector_config().candidate_speed_mps > profile.proof_relative_speed_mps


def test_irl_proof_drift_and_normal_are_independent_bounded_errors():
    from tejen_mission.m2_irl_launch_config import parse_gate_profiles

    payload = yaml.safe_load(CONFIG.read_text())
    payload["m2_irl"]["gate_profiles"]["attachment_search"]["proof_max_contact_drift_m"] = 0.040
    profile = parse_gate_profiles(payload).attachment_search
    assert profile.proof_max_contact_drift_m == 0.040
    assert profile.proof_normal_m == 0.015


def test_bottom_search_accepts_command_floor_while_face_is_still_above_contact_band():
    from tejen_mission.m2b_attachment_mission import M2BConfig, should_enter_irl_capture_search

    config = M2BConfig(capture_hold_normal_m=0.010)
    assert should_enter_irl_capture_search(
        config, measured_normal_m=0.019, commanded_height_m=0.0,
        measured_xy_m=0.020, evidence_fresh=True,
    )
    assert not should_enter_irl_capture_search(
        config, measured_normal_m=0.019, commanded_height_m=0.015,
        measured_xy_m=0.020, evidence_fresh=True,
    )
    assert not should_enter_irl_capture_search(
        config, measured_normal_m=0.019, commanded_height_m=0.0,
        measured_xy_m=0.020, evidence_fresh=False,
    )


def test_proof_rejects_magnet_that_follows_drone_even_after_motion_stops():
    from dataclasses import replace

    cfg = replace(default_detector_config(), candidate_normal_m=0.010,
                  proof_normal_m=0.025, proof_speed_mps=0.030,
                  proof_max_contact_drift_m=0.008)
    detector, plate = _observer(cfg)
    _sample(detector, plate, 0.0)
    assert _sample(detector, plate, 0.2).candidate_condition
    _sample(detector, plate, 0.25, proof=True)
    moved = (0.004, 0.0, 0.015)
    body = (0.005, 0.0, 0.020)
    _sample(detector, plate, 0.35, magnet_delta=moved, drone_delta=body, proof=True)
    _sample(detector, plate, 0.65, magnet_delta=moved, drone_delta=body, proof=True)
    result = _sample(detector, plate, 0.90, magnet_delta=moved, drone_delta=body, proof=True)
    assert result.proof_excitation_m >= cfg.proof_min_excitation_m
    assert result.relative_speed_mps < cfg.proof_speed_mps
    assert result.proof_contact_drift_m > cfg.proof_max_contact_drift_m
    assert not result.confirmed


def test_proof_accepts_plate_fixed_magnet_with_body_excitation():
    from dataclasses import replace

    cfg = replace(default_detector_config(), candidate_normal_m=0.010,
                  proof_normal_m=0.010, proof_speed_mps=0.030,
                  proof_max_contact_drift_m=0.008)
    detector, plate = _observer(cfg)
    _sample(detector, plate, 0.0)
    _sample(detector, plate, 0.2)
    _sample(detector, plate, 0.25, proof=True)
    body = (0.005, 0.0, 0.020)
    _sample(detector, plate, 0.45, drone_delta=body, proof=True)
    result = _sample(detector, plate, 0.70, drone_delta=body, proof=True)
    assert result.proof_contact_drift_m == 0.0
    assert result.confirmed


def test_stale_mocap_invalidates_one_proof_pull_instead_of_restarting_it():
    from dataclasses import replace

    cfg = replace(default_detector_config(), candidate_normal_m=0.010,
                  proof_normal_m=0.010, proof_speed_mps=0.030,
                  proof_max_contact_drift_m=0.008)
    detector, plate = _observer(cfg)
    _sample(detector, plate, 0.0)
    _sample(detector, plate, 0.2)
    _sample(detector, plate, 0.25, proof=True)
    assert not _sample(detector, plate, 0.55, proof=True, magnet_time_s=0.0).fresh
    _sample(detector, plate, 0.60, proof=True)
    _sample(detector, plate, 0.80, proof=True)
    body = (0.005, 0.0, 0.020)
    _sample(detector, plate, 0.85, drone_delta=body, proof=True)
    result = _sample(detector, plate, 1.10, drone_delta=body, proof=True)
    assert not result.confirmed



def test_irl_flush_bias_enters_contact_band_but_two_centimetre_air_gap_does_not():
    from dataclasses import replace

    from tejen_mission.m2_irl_launch_config import parse_gate_profiles
    from tejen_mission.m2b_attachment_mission import M2BConfig, should_enter_irl_capture_search

    profile = parse_gate_profiles(yaml.safe_load(CONFIG.read_text())).attachment_search
    mission = M2BConfig(capture_hold_normal_m=profile.capture_normal_m)
    assert should_enter_irl_capture_search(
        mission, measured_normal_m=-0.0108, commanded_height_m=0.0,
        measured_xy_m=0.010, evidence_fresh=True,
    )
    assert not should_enter_irl_capture_search(
        mission, measured_normal_m=-0.016, commanded_height_m=0.0,
        measured_xy_m=0.010, evidence_fresh=True,
    )
    cfg = replace(
        default_detector_config(),
        candidate_normal_m=profile.candidate_normal_m,
        proof_normal_m=profile.proof_normal_m,
        proof_max_contact_drift_m=profile.proof_max_contact_drift_m,
    )
    flush_detector, flush_plate = _observer(cfg)
    assert _sample(flush_detector, flush_plate, 0.0, magnet_delta=(0.0, 0.0, -0.0108)).candidate_condition
    high_detector, high_plate = _observer(cfg)
    assert not _sample(high_detector, high_plate, 0.0, magnet_delta=(0.0, 0.0, 0.020)).candidate_condition

def test_irl_uses_common_release_instead_of_failed_proof_exception():
    planner = (CONFIG.parent.parent / "tejen_mission" / "online_join_planner.py").read_text()
    launch = (CONFIG.parent.parent / "launch" / "m2_irl_two_drone.launch.py").read_text()
    assert "unconfirmed_clearance" not in planner
    assert "terminal_unconfirmed_retreat" not in planner
    assert '"m2b_irl_unconfirmed_clearance_rise_m":' not in launch
    recovery = planner.split("def update_m2b_progressive", 1)[1].split("def update_m2b_b1", 1)[0]
    assert "link_statistics_fresh(" in recovery
    assert "progressive_release_dwell_s" in recovery
    assert 'm2b_retreat_stage = "await_operator"' in recovery

def test_landing_requires_loss_only_after_a_confirmed_attachment():
    import ast
    import json
    from types import SimpleNamespace, MethodType

    path = CONFIG.parents[0].parent / "tejen_mission" / "online_join_planner.py"
    tree = ast.parse(path.read_text())
    names = {"m2b_attachment_diagnostics_callback", "m2b_geometry_separated"}
    methods = [node for node in ast.walk(tree)
               if isinstance(node, ast.FunctionDef) and node.name in names]
    module = ast.Module(body=ast.parse("from __future__ import annotations").body + methods,
                        type_ignores=[])
    namespace = {"json": json}
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    node = SimpleNamespace(m2b_attachment_truth_source="measured_detector",
                           m2b_attachment_was_confirmed=False,
                           m2b_assigned_plate_id=4, _physical_now_s=lambda: 1.0,
                           m2b_attachment_evidence_fresh=lambda now=None: True)
    callback = MethodType(namespace["m2b_attachment_diagnostics_callback"], node)
    separated = MethodType(namespace["m2b_geometry_separated"], node)
    def observe(**values):
        callback(SimpleNamespace(data=json.dumps(dict(
            assigned_plate_id=4, confirmed=False, lost=False,
            geometry_separated=True, **values))))
    observe()
    assert separated(require_lost=True)
    node.m2b_attachment_evidence_fresh = lambda now=None: False
    assert not separated(require_lost=True)
    node.m2b_attachment_evidence_fresh = lambda now=None: True
    callback(SimpleNamespace(data=json.dumps(dict(assigned_plate_id=4,
        confirmed=True, lost=False, geometry_separated=False))))
    observe()  # Detector reset/current false must not erase confirmed history.
    assert not separated(require_lost=True)
    node.m2b_attachment_diagnostics["lost"] = True
    assert separated(require_lost=True)
    node.m2b_attachment_diagnostics["geometry_separated"] = False
    assert not separated(require_lost=True)


def _contact_planner_method(name, node):
    import ast
    import math
    from types import MethodType
    from tejen_mission.mission_types import MissionPhase
    from tejen_mission.m2b_attachment_mission import body_reference_from_contact, contact_capture_position
    path = CONFIG.parent.parent / "tejen_mission" / "online_join_planner.py"
    tree = ast.parse(path.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=ast.parse("from __future__ import annotations").body + [method], type_ignores=[])
    namespace = dict(np=np, math=math, MissionPhase=MissionPhase,
                     body_reference_from_contact=body_reference_from_contact,
                     contact_capture_position=contact_capture_position)
    exec(compile(ast.fix_missing_locations(module), str(path), "exec"), namespace)
    return MethodType(namespace[name], node)


def test_irl_descent_reaches_bounded_negative_floor_without_early_freeze():
    from types import SimpleNamespace
    from tejen_mission.m2b_attachment_mission import M2BConfig, M2BDescentProgress, should_enter_irl_capture_search
    cfg = M2BConfig(contact_overtravel_m=0.005)
    descent = M2BDescentProgress(cfg, initial_contact_height_m=0.003)
    for i in range(20):
        result = descent.step(now_s=i / 30, measured_xy_error_m=0.01, dt_s=1 / 30)
        assert result.target_contact_height_m >= -0.005
    assert result.target_contact_height_m == -0.005
    assert M2BConfig().contact_height_floor_m == 0.0
    assert not should_enter_irl_capture_search(cfg, measured_normal_m=0.009,
        commanded_height_m=0.0, measured_xy_m=0.01, evidence_fresh=True)
    assert should_enter_irl_capture_search(cfg, measured_normal_m=0.009,
        commanded_height_m=-0.005, measured_xy_m=0.01, evidence_fresh=True)
    node = SimpleNamespace(m2b_config=cfg,
        m2b_plate_pose_world=lambda: (np.zeros(3), np.eye(3)),
        m2b_anchor_to_contact_length_m=0.565,
        m2b_tether_anchor_body=np.array([0.0, 0.0, -0.04]),
        m2b_body_rotation_world=lambda: np.eye(3))
    target = _contact_planner_method("m2b_body_target_for_contact_height", node)
    assert np.isclose(target(-0.005)[2], 0.600)
    assert np.isclose(target(-0.020)[2], 0.600)
    node.m2b_config = M2BConfig()
    assert np.isclose(target(-0.005)[2], 0.605)


def test_irl_preproof_dwell_requires_surface_contact_even_with_ready_candidate():
    from types import SimpleNamespace
    from tejen_mission.mission_types import MissionPhase
    from tejen_mission.m2b_attachment_mission import M2BConfig
    node = SimpleNamespace(m2b_attachment_truth_source="measured_detector",
        m2b_irl_capture_search_enabled=True, phase=MissionPhase.M2_ATTACH_CAPTURE_WAIT,
        m2b_config=M2BConfig(), m2b_attachment_evidence_fresh=lambda now: True,
        m2b_attachment_diagnostics=dict(candidate_ready=True, confirmed=False, lost=False, normal_error_m=0.009))
    latched = _contact_planner_method("m2b_physical_latched", node)
    assert not latched()
    node.m2b_attachment_diagnostics["normal_error_m"] = 0.0
    assert latched()
    node.m2b_attachment_evidence_fresh = lambda now: False
    assert not latched()
    node.m2b_attachment_evidence_fresh = lambda now: True
    node.phase = MissionPhase.M2_ATTACH_PROOF
    node.m2b_attachment_diagnostics.update(normal_error_m=0.009, confirmed=True)
    assert latched()
