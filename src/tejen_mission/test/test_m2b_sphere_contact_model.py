"""Test-first contracts for M2B's simulation sphere-centre contact observation.

The real/rigid calibrated contact-frame detector remains the default.  M2B simulation
may explicitly select a sphere-centre model so arbitrary ball spin cannot fabricate
contact-point motion or normal separation.
"""

from __future__ import annotations

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from tejen_mission.attachment_detection import (
    AttachmentDetector,
    AttachmentDetectorConfig,
    AttachmentObservation,
    MagnetContactCalibration,
    MagnetContactObservationModel,
)
from tejen_mission.cooperative_trajectory import RingNetGeometry


ROOT = Path(__file__).resolve().parents[3]
PKG = ROOT / "src" / "tejen_mission"
LAUNCH = PKG / "launch" / "m2b_b1_single_attachment.launch.py"
OBSERVER = PKG / "tejen_mission" / "m2_attachment_observer.py"
MANAGER = PKG / "tejen_mission" / "m2a_physical_capture_manager.py"
REPLAY = PKG / "tejen_mission" / "m2a_attachment_replay.py"
X3 = ROOT / "simulation_assets" / "tejen" / "modelLargeM2BallMagnet.sdf"


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _config(*, tau: float = 0.0) -> AttachmentDetectorConfig:
    return AttachmentDetectorConfig(
        candidate_xy_m=0.020,
        candidate_normal_m=0.020,
        candidate_speed_mps=0.12,
        candidate_dwell_s=0.10,
        proof_xy_m=0.025,
        proof_normal_m=0.025,
        proof_speed_mps=0.15,
        proof_dwell_s=0.10,
        proof_min_excitation_m=0.012,
        proof_direction_plate=np.array([1.0, 0.0, 0.0]),
        loss_xy_m=0.045,
        loss_normal_m=0.045,
        loss_dwell_s=0.12,
        pose_timeout_s=0.25,
        velocity_filter_tau_s=tau,
    )


def _calibration() -> MagnetContactCalibration:
    return MagnetContactCalibration(
        contact_position_magnet=np.array([0.0, 0.0, -0.025]),
        rotation_magnet_from_contact=np.eye(3),
    )


def _plate_pose(geometry: RingNetGeometry, plate_id: int = 0):
    ring_position = np.array([0.4, -0.3, 0.2])
    ring_rotation = _rz(0.7) @ _ry(-0.25) @ _rx(0.18)
    p_plate = geometry.plate_position_world(
        ring_position=ring_position,
        rotation_world_from_ring=ring_rotation,
        plate_index=plate_id,
    )
    r_plate = geometry.plate_rotation_world(
        rotation_world_from_ring=ring_rotation,
        plate_index=plate_id,
    )
    return ring_position, ring_rotation, p_plate, r_plate


def _observation(*, t: float, center_plate: np.ndarray, magnet_rotation: np.ndarray, magnet_on=True):
    geometry = RingNetGeometry()
    ring_position, ring_rotation, p_plate, r_plate = _plate_pose(geometry)
    p_magnet = p_plate + r_plate @ np.asarray(center_plate, dtype=float)
    p_drone = p_plate + r_plate @ np.array([0.0, 0.0, 0.50])
    return AttachmentObservation(
        time_s=t,
        ring_position_world=ring_position,
        rotation_world_from_ring=ring_rotation,
        ring_time_s=t,
        magnet_position_world=p_magnet,
        rotation_world_from_magnet=magnet_rotation,
        magnet_time_s=t,
        drone_position_world=p_drone,
        drone_time_s=t,
        magnet_on=magnet_on,
        proof_requested=False,
    )


def _sphere_detector(*, tau: float = 0.0) -> AttachmentDetector:
    return AttachmentDetector(
        geometry=RingNetGeometry(),
        assigned_plate_id=0,
        calibration=_calibration(),
        config=_config(tau=tau),
        contact_observation_model=MagnetContactObservationModel.SPHERE_CENTER,
        sphere_radius_m=0.025,
    )


def test_sphere_center_geometry_is_invariant_to_arbitrary_ball_spin():
    center_plate = np.array([0.004, -0.003, 0.028])  # 3 mm physical surface gap
    outputs = []
    for i, rotation in enumerate((np.eye(3), _rx(1.3), _ry(-2.1) @ _rz(0.8))):
        detector = _sphere_detector()
        outputs.append(detector.update(_observation(t=1.0 + i, center_plate=center_plate, magnet_rotation=rotation)))

    for out in outputs:
        np.testing.assert_allclose(out.plate_relative_position_m, [0.004, -0.003, 0.003], atol=1e-12)
        assert abs(out.xy_error_m - 0.005) < 1e-12
        assert abs(out.normal_error_m - 0.003) < 1e-12
        assert out.orientation_error_rad == 0.0


def test_sphere_center_relative_speed_ignores_spin_but_detects_center_translation():
    detector = _sphere_detector()
    first = detector.update(_observation(t=0.0, center_plate=np.array([0.0, 0.0, 0.028]), magnet_rotation=np.eye(3)))
    spun = detector.update(_observation(t=0.1, center_plate=np.array([0.0, 0.0, 0.028]), magnet_rotation=_rx(2.4) @ _rz(1.7)))
    moved = detector.update(_observation(t=0.2, center_plate=np.array([0.006, 0.0, 0.028]), magnet_rotation=_ry(-1.9)))

    assert first.relative_speed_mps == 0.0
    assert spun.relative_speed_mps == 0.0
    assert abs(moved.relative_speed_mps - 0.06) < 1e-9


def test_sphere_center_candidate_can_accumulate_dwell_while_ball_spins():
    detector = _sphere_detector()
    outputs = []
    for i, t in enumerate((0.0, 0.05, 0.10, 0.15)):
        outputs.append(
            detector.update(
                _observation(
                    t=t,
                    center_plate=np.array([0.001, -0.001, 0.029]),
                    magnet_rotation=_rx(0.9 * i) @ _rz(1.1 * i),
                )
            )
        )
    assert all(out.candidate_condition for out in outputs)
    assert outputs[-1].candidate_dwell_s >= 0.10
    assert outputs[-1].relative_speed_mps == 0.0


def test_default_rigid_calibrated_mode_remains_orientation_sensitive_for_irl():
    geometry = RingNetGeometry()
    rigid = AttachmentDetector(
        geometry=geometry,
        assigned_plate_id=0,
        calibration=_calibration(),
        config=_config(),
    )
    center = np.array([0.0, 0.0, 0.025])
    _ring_position, _ring_rotation, _p_plate, r_plate = _plate_pose(geometry)
    a = rigid.update(_observation(t=0.0, center_plate=center, magnet_rotation=r_plate))
    b = rigid.update(
        _observation(t=0.1, center_plate=center, magnet_rotation=r_plate @ _rx(math.pi / 2.0))
    )
    assert abs(a.normal_error_m) < 1e-12
    assert b.xy_error_m > 0.020


def test_b1_launch_explicitly_selects_sphere_center_for_detector_and_physical_capture():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("contact_observation_model", default_value="sphere_center")' in text
    assert 'DeclareLaunchArgument("magnet_sphere_radius_m", default_value="0.025")' in text
    assert text.count('"contact_observation_model": contact_observation_model') >= 2
    assert text.count('"sphere_radius_m": ParameterValue(magnet_sphere_radius_m, value_type=float)') >= 2


def test_simulation_sphere_radius_parameter_matches_current_real_x3_model():
    root = ET.parse(X3).getroot()
    radius = float(root.find("./model/link[@name='magnet_tip_link']/collision/geometry/sphere/radius").text)
    assert abs(radius - 0.025) < 1e-12


def test_observer_and_b1_telemetry_expose_contact_model_and_center_geometry():
    observer = OBSERVER.read_text(encoding="utf-8")
    assert '"contact_observation_model"' in observer
    assert '"magnet_center_xy_error_m"' in observer
    assert '"magnet_center_normal_m"' in observer
    assert '"magnet_surface_gap_m"' in observer
    telemetry = (PKG / "tejen_mission" / "m2b_b1_telemetry.py").read_text(encoding="utf-8")
    for field in (
        '"detector_contact_observation_model"',
        '"magnet_center_xy_error_m"',
        '"magnet_center_normal_m"',
        '"magnet_surface_gap_m"',
    ):
        assert field in telemetry


def test_physical_capture_manager_has_explicit_sphere_center_mode_without_using_ball_rotation_for_capture_position():
    text = MANAGER.read_text(encoding="utf-8")
    assert 'declare_parameter("contact_observation_model", "rigid_calibrated")' in text
    assert 'declare_parameter("sphere_radius_m", 0.025)' in text
    assert 'if self.contact_observation_model == "sphere_center":' in text
    sphere_branch = text.split('if self.contact_observation_model == "sphere_center":', 1)[1].split("else:", 1)[0]
    assert "p_m" in sphere_branch
    assert "R_m @ self.contact_offset" not in sphere_branch
    assert "self.sphere_radius_m" in sphere_branch


def test_offline_replay_restores_contact_observation_model_from_metadata():
    text = REPLAY.read_text(encoding="utf-8")
    assert 'contact_observation_model' in text
    assert 'sphere_radius_m' in text
    assert 'AttachmentDetector(' in text


def test_b1_capture_wait_gap_and_physical_capture_gap_share_one_launch_parameter():
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("magnet_capture_gap_m", default_value="0.010")' in text
    assert '"physical_capture_normal_m": ParameterValue(magnet_capture_gap_m, value_type=float)' in text
    assert '"m2b_capture_hold_normal_m": ParameterValue(magnet_capture_gap_m, value_type=float)' in text


def test_b1_runner_makes_simulation_contact_model_explicit_at_launch_boundary():
    text = (ROOT / "tools" / "sim_test" / "run_m2b_b1.sh").read_text(encoding="utf-8")
    assert 'CONTACT_OBSERVATION_MODEL="${M2B_CONTACT_OBSERVATION_MODEL:-sphere_center}"' in text
    assert 'MAGNET_SPHERE_RADIUS_M="${M2B_MAGNET_SPHERE_RADIUS_M:-0.025}"' in text
    assert 'contact_observation_model:="$CONTACT_OBSERVATION_MODEL"' in text
    assert 'magnet_sphere_radius_m:="$MAGNET_SPHERE_RADIUS_M"' in text


def test_offline_replay_prefers_attachment_specific_metadata_for_b1_logs():
    text = REPLAY.read_text(encoding="utf-8")
    assert 'attachment_metadata.json' in text
    assert text.index('attachment_metadata.json') < text.index('metadata.json')


def test_b1_metadata_records_sphere_radius_and_shared_capture_gap():
    launch = LAUNCH.read_text(encoding="utf-8")
    telemetry = (PKG / "tejen_mission" / "m2b_b1_telemetry.py").read_text(encoding="utf-8")
    assert '"magnet_capture_gap_m": ParameterValue(magnet_capture_gap_m, value_type=float)' in launch
    assert 'self.magnet_capture_gap_m' in telemetry
    assert '"magnet_capture_gap_m": self.magnet_capture_gap_m' in telemetry
