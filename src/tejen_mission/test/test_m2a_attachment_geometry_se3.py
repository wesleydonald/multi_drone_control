"""Deterministic SE(3) validation for the pure M2A attachment detector.

This suite deliberately has no ROS, Gazebo, SDF, controller, or planner dependency.
It validates the geometric contract

    T_P^C = inverse(T_W^P) T_W^M T_M^C

using independently constructed plate/contact poses, including arbitrary 3-D
world transforms and non-trivial magnet-frame calibration.

All numeric limits here are unit-test fixtures, not commissioning thresholds.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from tejen_mission.attachment_detection import (
    AttachmentDetector,
    AttachmentDetectorConfig,
    AttachmentObservation,
    MagnetContactCalibration,
)
from tejen_mission.cooperative_trajectory import RingNetGeometry


ATOL = 2e-9


def _rx(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float
    )


def _ry(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float
    )


def _rz(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array(
        [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float
    )


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    # Independent ZYX construction: R_WB = Rz(yaw) Ry(pitch) Rx(roll).
    return _rz(yaw) @ _ry(pitch) @ _rx(roll)


def _independent_plate_pose(
    geometry: RingNetGeometry,
    plate_index: int,
    *,
    ring_position_world: np.ndarray,
    rotation_world_from_ring: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Construct expected T_W^P without production plate-frame helpers."""

    theta = (
        float(geometry.angle_zero_rad)
        + 2.0 * math.pi * int(plate_index) / int(geometry.plate_count)
    )
    radius = 0.5 * float(geometry.plate_pitch_diameter_m)
    p_rp = np.array([radius * math.cos(theta), radius * math.sin(theta), 0.0])

    # Plate frame convention under test:
    # x_P radial outward, y_P tangent in increasing-index direction, z_P ring normal.
    r_rp = _rz(theta)
    p_wp = np.asarray(ring_position_world, dtype=float) + rotation_world_from_ring @ p_rp
    r_wp = rotation_world_from_ring @ r_rp
    return p_wp, r_wp


def _magnet_measurement_from_contact_pose(
    *,
    contact_position_world: np.ndarray,
    rotation_world_from_contact: np.ndarray,
    contact_position_magnet: np.ndarray,
    rotation_magnet_from_contact: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Recover T_W^M from desired T_W^C and fixed T_M^C, independently."""

    # R_WC = R_WM R_MC  ->  R_WM = R_WC R_MC^T
    r_wm = rotation_world_from_contact @ rotation_magnet_from_contact.T
    # p_WC = p_WM + R_WM p_MC
    p_wm = contact_position_world - r_wm @ contact_position_magnet
    return p_wm, r_wm


def _config(*, velocity_filter_tau_s: float = 0.0) -> AttachmentDetectorConfig:
    proof_angle = math.radians(15.0)
    return AttachmentDetectorConfig(
        candidate_xy_m=0.020,
        candidate_normal_m=0.015,
        candidate_speed_mps=0.30,
        candidate_dwell_s=0.10,
        proof_xy_m=0.025,
        proof_normal_m=0.020,
        proof_speed_mps=0.35,
        proof_dwell_s=0.10,
        proof_min_excitation_m=0.030,
        proof_direction_plate=np.array(
            [math.sin(proof_angle), 0.0, math.cos(proof_angle)], dtype=float
        ),
        loss_xy_m=0.050,
        loss_normal_m=0.040,
        loss_dwell_s=0.15,
        pose_timeout_s=0.10,
        velocity_filter_tau_s=velocity_filter_tau_s,
    )


def _calibration(
    *,
    contact_position_magnet=(0.0, 0.0, 0.0),
    rotation_magnet_from_contact: np.ndarray | None = None,
) -> MagnetContactCalibration:
    if rotation_magnet_from_contact is None:
        rotation_magnet_from_contact = np.eye(3)
    return MagnetContactCalibration(
        contact_position_magnet=np.asarray(contact_position_magnet, dtype=float),
        rotation_magnet_from_contact=np.asarray(rotation_magnet_from_contact, dtype=float),
    )


def _detector(
    *,
    plate_index: int,
    calibration: MagnetContactCalibration | None = None,
    config: AttachmentDetectorConfig | None = None,
) -> AttachmentDetector:
    return AttachmentDetector(
        geometry=RingNetGeometry(),
        assigned_plate_id=plate_index,
        calibration=_calibration() if calibration is None else calibration,
        config=_config() if config is None else config,
    )


def _observation_from_plate_relative_contact(
    *,
    t: float,
    physical_plate_index: int,
    ring_position_world: np.ndarray,
    rotation_world_from_ring: np.ndarray,
    contact_position_plate: np.ndarray,
    calibration: MagnetContactCalibration,
    drone_position_plate: np.ndarray = np.array([0.0, 0.0, 0.50]),
    magnet_on: bool = True,
    proof_requested: bool = False,
) -> AttachmentObservation:
    geometry = RingNetGeometry()
    p_wp, r_wp = _independent_plate_pose(
        geometry,
        physical_plate_index,
        ring_position_world=ring_position_world,
        rotation_world_from_ring=rotation_world_from_ring,
    )

    p_pc = np.asarray(contact_position_plate, dtype=float)
    p_wc = p_wp + r_wp @ p_pc
    r_wc = r_wp
    p_wm, r_wm = _magnet_measurement_from_contact_pose(
        contact_position_world=p_wc,
        rotation_world_from_contact=r_wc,
        contact_position_magnet=calibration.contact_position_magnet,
        rotation_magnet_from_contact=calibration.rotation_magnet_from_contact,
    )
    p_wq = p_wp + r_wp @ np.asarray(drone_position_plate, dtype=float)

    return AttachmentObservation(
        time_s=float(t),
        ring_position_world=np.asarray(ring_position_world, dtype=float),
        rotation_world_from_ring=np.asarray(rotation_world_from_ring, dtype=float),
        ring_time_s=float(t),
        magnet_position_world=p_wm,
        rotation_world_from_magnet=r_wm,
        magnet_time_s=float(t),
        drone_position_world=p_wq,
        drone_time_s=float(t),
        magnet_on=bool(magnet_on),
        proof_requested=bool(proof_requested),
    )


def _assert_geometry_output(output, expected_position_plate: np.ndarray) -> None:
    expected = np.asarray(expected_position_plate, dtype=float)
    np.testing.assert_allclose(output.plate_relative_position_m, expected, atol=ATOL, rtol=0.0)
    assert output.radial_error_m == pytest.approx(expected[0], abs=ATOL)
    assert output.tangential_error_m == pytest.approx(expected[1], abs=ATOL)
    assert output.normal_error_m == pytest.approx(expected[2], abs=ATOL)
    assert output.xy_error_m == pytest.approx(float(np.linalg.norm(expected[:2])), abs=ATOL)


@pytest.mark.parametrize("plate_index", range(12))
def test_exact_contact_is_zero_for_all_twelve_plates(plate_index: int) -> None:
    calibration = _calibration()
    detector = _detector(plate_index=plate_index, calibration=calibration)
    observation = _observation_from_plate_relative_contact(
        t=0.0,
        physical_plate_index=plate_index,
        ring_position_world=np.zeros(3),
        rotation_world_from_ring=np.eye(3),
        contact_position_plate=np.zeros(3),
        calibration=calibration,
    )

    output = detector.update(observation)
    _assert_geometry_output(output, np.zeros(3))
    assert output.orientation_error_rad == pytest.approx(0.0, abs=ATOL)


@pytest.mark.parametrize(
    "offset_plate",
    [
        np.array([+0.010, 0.0, 0.0]),
        np.array([-0.010, 0.0, 0.0]),
        np.array([0.0, +0.010, 0.0]),
        np.array([0.0, -0.010, 0.0]),
        np.array([0.0, 0.0, +0.010]),
        np.array([0.0, 0.0, -0.010]),
    ],
    ids=["radial_plus", "radial_minus", "tangent_plus", "tangent_minus", "normal_plus", "normal_minus"],
)
def test_signed_plate_axis_perturbations_are_recovered_exactly(offset_plate: np.ndarray) -> None:
    calibration = _calibration()
    detector = _detector(plate_index=4, calibration=calibration)
    observation = _observation_from_plate_relative_contact(
        t=0.0,
        physical_plate_index=4,
        ring_position_world=np.array([0.3, -0.2, 0.7]),
        rotation_world_from_ring=_rpy(0.17, -0.11, 0.83),
        contact_position_plate=offset_plate,
        calibration=calibration,
    )

    output = detector.update(observation)
    _assert_geometry_output(output, offset_plate)


@pytest.mark.parametrize(
    "translation",
    [
        np.array([1.2, -0.7, 0.4]),
        np.array([-2.0, 1.1, 0.2]),
    ],
)
def test_global_translation_does_not_change_plate_relative_geometry(translation: np.ndarray) -> None:
    plate_index = 7
    relative = np.array([0.013, -0.009, 0.006])
    calibration = _calibration()

    baseline = _detector(plate_index=plate_index, calibration=calibration).update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=plate_index,
            ring_position_world=np.zeros(3),
            rotation_world_from_ring=np.eye(3),
            contact_position_plate=relative,
            calibration=calibration,
        )
    )
    moved = _detector(plate_index=plate_index, calibration=calibration).update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=plate_index,
            ring_position_world=translation,
            rotation_world_from_ring=np.eye(3),
            contact_position_plate=relative,
            calibration=calibration,
        )
    )

    np.testing.assert_allclose(
        moved.plate_relative_position_m,
        baseline.plate_relative_position_m,
        atol=ATOL,
        rtol=0.0,
    )


@pytest.mark.parametrize(
    "rotation_world_from_ring",
    [
        _rpy(0.0, 0.0, math.radians(123.0)),
        _rpy(math.radians(17.0), math.radians(-11.0), 0.0),
        _rpy(math.radians(-23.0), math.radians(14.0), math.radians(71.0)),
    ],
    ids=["yaw", "roll_pitch", "full_rpy"],
)
def test_global_rotation_does_not_change_plate_relative_geometry(
    rotation_world_from_ring: np.ndarray,
) -> None:
    plate_index = 9
    relative = np.array([-0.012, 0.008, -0.004])
    calibration = _calibration()

    baseline = _detector(plate_index=plate_index, calibration=calibration).update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=plate_index,
            ring_position_world=np.array([0.4, -0.1, 0.5]),
            rotation_world_from_ring=np.eye(3),
            contact_position_plate=relative,
            calibration=calibration,
        )
    )
    rotated = _detector(plate_index=plate_index, calibration=calibration).update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=plate_index,
            ring_position_world=np.array([0.4, -0.1, 0.5]),
            rotation_world_from_ring=rotation_world_from_ring,
            contact_position_plate=relative,
            calibration=calibration,
        )
    )

    np.testing.assert_allclose(
        rotated.plate_relative_position_m,
        baseline.plate_relative_position_m,
        atol=ATOL,
        rtol=0.0,
    )


@pytest.mark.parametrize("plate_index", range(12))
@pytest.mark.parametrize("yaw_deg", range(0, 360, 30))
def test_exact_contact_survives_full_basket_yaw_sweep(plate_index: int, yaw_deg: int) -> None:
    calibration = _calibration()
    detector = _detector(plate_index=plate_index, calibration=calibration)
    output = detector.update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=plate_index,
            ring_position_world=np.array([-0.8, 0.6, 0.2]),
            rotation_world_from_ring=_rz(math.radians(float(yaw_deg))),
            contact_position_plate=np.zeros(3),
            calibration=calibration,
        )
    )

    _assert_geometry_output(output, np.zeros(3))


@pytest.mark.parametrize(
    "roll_deg,pitch_deg,yaw_deg",
    [
        (+5.0, 0.0, 0.0),
        (-5.0, 0.0, 0.0),
        (0.0, +5.0, 0.0),
        (0.0, -5.0, 0.0),
        (+5.0, -5.0, 37.0),
        (-4.0, +3.0, -81.0),
    ],
)
def test_exact_contact_survives_small_3d_ring_attitude(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> None:
    calibration = _calibration()
    detector = _detector(plate_index=3, calibration=calibration)
    output = detector.update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=3,
            ring_position_world=np.array([0.2, 0.1, 0.15]),
            rotation_world_from_ring=_rpy(
                math.radians(roll_deg),
                math.radians(pitch_deg),
                math.radians(yaw_deg),
            ),
            contact_position_plate=np.zeros(3),
            calibration=calibration,
        )
    )

    _assert_geometry_output(output, np.zeros(3))


@pytest.mark.parametrize(
    "contact_position_magnet,rotation_magnet_from_contact",
    [
        (np.array([0.012, -0.007, -0.025]), np.eye(3)),
        (np.zeros(3), _rpy(math.radians(12.0), math.radians(-8.0), math.radians(21.0))),
        (
            np.array([0.015, 0.006, -0.022]),
            _rpy(math.radians(-16.0), math.radians(9.0), math.radians(33.0)),
        ),
    ],
    ids=["translation", "rotation", "translation_and_rotation"],
)
def test_nontrivial_mocap_to_contact_calibration_is_respected(
    contact_position_magnet: np.ndarray,
    rotation_magnet_from_contact: np.ndarray,
) -> None:
    calibration = _calibration(
        contact_position_magnet=contact_position_magnet,
        rotation_magnet_from_contact=rotation_magnet_from_contact,
    )
    relative = np.array([0.007, -0.005, 0.004])
    detector = _detector(plate_index=10, calibration=calibration)
    output = detector.update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=10,
            ring_position_world=np.array([1.1, -0.4, 0.3]),
            rotation_world_from_ring=_rpy(0.19, -0.13, 1.27),
            contact_position_plate=relative,
            calibration=calibration,
        )
    )

    _assert_geometry_output(output, relative)
    assert output.orientation_error_rad == pytest.approx(0.0, abs=ATOL)


@pytest.mark.parametrize("assigned_plate", range(1, 12))
def test_wrong_plate_assignment_cannot_look_like_contact(assigned_plate: int) -> None:
    calibration = _calibration()
    detector = _detector(plate_index=assigned_plate, calibration=calibration)
    output = detector.update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=0,
            ring_position_world=np.array([0.6, -0.5, 0.3]),
            rotation_world_from_ring=_rpy(0.08, -0.06, 0.72),
            contact_position_plate=np.zeros(3),
            calibration=calibration,
            magnet_on=True,
        )
    )

    assert not output.candidate_condition
    assert output.xy_error_m > detector.config.candidate_xy_m


@pytest.mark.parametrize(
    "ring_position_world,rotation_world_from_ring",
    [
        (np.zeros(3), np.eye(3)),
        (np.array([1.0, -0.6, 0.4]), _rz(1.1)),
        (np.array([-0.7, 0.3, 0.2]), _rpy(0.14, -0.09, -1.4)),
    ],
    ids=["identity", "translated_yawed", "full_se3"],
)
def test_proof_confirmation_is_invariant_to_world_pose(
    ring_position_world: np.ndarray,
    rotation_world_from_ring: np.ndarray,
) -> None:
    calibration = _calibration(
        contact_position_magnet=np.array([0.011, -0.004, -0.023]),
        rotation_magnet_from_contact=_rpy(0.09, -0.05, 0.18),
    )
    detector = _detector(plate_index=6, calibration=calibration)
    direction = detector.config.proof_direction_plate
    base_drone = np.array([0.0, 0.0, 0.50])

    output = None
    # Acquire candidate dwell at exact contact.
    for t in (0.0, 0.05, 0.11):
        output = detector.update(
            _observation_from_plate_relative_contact(
                t=t,
                physical_plate_index=6,
                ring_position_world=ring_position_world,
                rotation_world_from_ring=rotation_world_from_ring,
                contact_position_plate=np.zeros(3),
                calibration=calibration,
                drone_position_plate=base_drone,
                magnet_on=True,
                proof_requested=False,
            )
        )
    assert output is not None and output.candidate_condition

    # Begin proof, then move vehicle 35 mm along approved plate-frame direction
    # while the magnet contact remains registered to the plate.
    detector.update(
        _observation_from_plate_relative_contact(
            t=0.12,
            physical_plate_index=6,
            ring_position_world=ring_position_world,
            rotation_world_from_ring=rotation_world_from_ring,
            contact_position_plate=np.zeros(3),
            calibration=calibration,
            drone_position_plate=base_drone,
            magnet_on=True,
            proof_requested=True,
        )
    )
    proof_drone = base_drone + 0.035 * direction
    detector.update(
        _observation_from_plate_relative_contact(
            t=0.18,
            physical_plate_index=6,
            ring_position_world=ring_position_world,
            rotation_world_from_ring=rotation_world_from_ring,
            contact_position_plate=np.zeros(3),
            calibration=calibration,
            drone_position_plate=proof_drone,
            magnet_on=True,
            proof_requested=True,
        )
    )
    output = detector.update(
        _observation_from_plate_relative_contact(
            t=0.23,
            physical_plate_index=6,
            ring_position_world=ring_position_world,
            rotation_world_from_ring=rotation_world_from_ring,
            contact_position_plate=np.zeros(3),
            calibration=calibration,
            drone_position_plate=proof_drone,
            magnet_on=True,
            proof_requested=True,
        )
    )

    assert output.confirmed
    assert output.proof_excitation_m == pytest.approx(0.035, abs=ATOL)
    _assert_geometry_output(output, np.zeros(3))


def test_rigidly_registered_magnet_has_zero_relative_velocity_during_translation_and_yaw() -> None:
    calibration = _calibration()
    detector = _detector(
        plate_index=2,
        calibration=calibration,
        config=_config(velocity_filter_tau_s=0.0),
    )

    outputs = []
    for t in (0.0, 0.05, 0.10, 0.15, 0.20):
        ring_position = np.array([0.4 * t, -0.2 * t, 0.3 + 0.1 * t])
        ring_rotation = _rz(0.8 * t)
        outputs.append(
            detector.update(
                _observation_from_plate_relative_contact(
                    t=t,
                    physical_plate_index=2,
                    ring_position_world=ring_position,
                    rotation_world_from_ring=ring_rotation,
                    contact_position_plate=np.zeros(3),
                    calibration=calibration,
                )
            )
        )

    for output in outputs:
        np.testing.assert_allclose(
            output.plate_relative_velocity_mps,
            np.zeros(3),
            atol=2e-8,
            rtol=0.0,
        )


def test_known_plate_relative_motion_recovers_known_velocity() -> None:
    calibration = _calibration()
    detector = _detector(
        plate_index=5,
        calibration=calibration,
        config=_config(velocity_filter_tau_s=0.0),
    )
    expected_velocity = np.array([0.040, -0.025, 0.015])

    first = detector.update(
        _observation_from_plate_relative_contact(
            t=0.0,
            physical_plate_index=5,
            ring_position_world=np.array([0.5, -0.3, 0.2]),
            rotation_world_from_ring=_rpy(0.11, -0.07, 0.9),
            contact_position_plate=np.array([0.003, -0.002, 0.001]),
            calibration=calibration,
        )
    )
    np.testing.assert_allclose(first.plate_relative_velocity_mps, np.zeros(3), atol=ATOL)

    dt = 0.08
    output = detector.update(
        _observation_from_plate_relative_contact(
            t=dt,
            physical_plate_index=5,
            ring_position_world=np.array([0.5, -0.3, 0.2]),
            rotation_world_from_ring=_rpy(0.11, -0.07, 0.9),
            contact_position_plate=np.array([0.003, -0.002, 0.001]) + expected_velocity * dt,
            calibration=calibration,
        )
    )

    np.testing.assert_allclose(
        output.plate_relative_velocity_mps,
        expected_velocity,
        atol=2e-9,
        rtol=0.0,
    )


def test_plate_rotations_are_proper_and_follow_axis_convention() -> None:
    geometry = RingNetGeometry()

    for plate_index in range(geometry.plate_count):
        rotation = geometry.plate_rotation_body(plate_index)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=ATOL, rtol=0.0)
        assert np.linalg.det(rotation) == pytest.approx(1.0, abs=ATOL)

        theta = geometry.angle_zero_rad + 2.0 * math.pi * plate_index / geometry.plate_count
        expected_radial = np.array([math.cos(theta), math.sin(theta), 0.0])
        expected_tangent = np.array([-math.sin(theta), math.cos(theta), 0.0])
        expected_normal = np.array([0.0, 0.0, 1.0])
        np.testing.assert_allclose(rotation[:, 0], expected_radial, atol=ATOL, rtol=0.0)
        np.testing.assert_allclose(rotation[:, 1], expected_tangent, atol=ATOL, rtol=0.0)
        np.testing.assert_allclose(rotation[:, 2], expected_normal, atol=ATOL, rtol=0.0)


def test_plate_angular_spacing_is_exactly_thirty_degrees() -> None:
    geometry = RingNetGeometry()
    positions = [geometry.plate_body_offset(i) for i in range(geometry.plate_count)]
    angles = [math.atan2(p[1], p[0]) % (2.0 * math.pi) for p in positions]
    angles.sort()
    wrapped = angles + [angles[0] + 2.0 * math.pi]
    spacings = np.diff(wrapped)
    np.testing.assert_allclose(
        spacings,
        np.full(geometry.plate_count, math.radians(30.0)),
        atol=ATOL,
        rtol=0.0,
    )
