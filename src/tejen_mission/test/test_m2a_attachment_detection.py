"""Test-first contract for the M2A attachment detector.

These tests intentionally target production APIs that do not exist yet.  The
first M2A implementation must make them green without importing ROS/Gazebo into
``attachment_detection.py`` or using Gazebo joint truth as detector input.

All numerical thresholds here are unit-test fixtures chosen to make behaviour
unambiguous.  They are NOT commissioning values for simulation or IRL.
"""

from __future__ import annotations

import ast
from dataclasses import replace
from dataclasses import fields
from importlib import import_module
from pathlib import Path
import math

import numpy as np
import pytest

from tejen_mission.cooperative_trajectory import RingNetGeometry


MODULE_NAME = "tejen_mission.attachment_detection"


def _api():
    try:
        return import_module(MODULE_NAME)
    except ModuleNotFoundError as exc:
        pytest.fail(
            "M2A test-first RED: production module "
            "tejen_mission/attachment_detection.py does not exist yet. "
            "Implement it only after these behavioural contracts are reviewed.",
            pytrace=False,
        )


def _rz(yaw: float) -> np.ndarray:
    c = math.cos(float(yaw))
    s = math.sin(float(yaw))
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _rx(angle: float) -> np.ndarray:
    c = math.cos(float(angle))
    s = math.sin(float(angle))
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def _plate_pose(
    geometry: RingNetGeometry,
    plate_index: int,
    *,
    ring_position: np.ndarray | None = None,
    rotation_world_from_ring: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    ring_position = (
        np.zeros(3, dtype=float)
        if ring_position is None
        else np.asarray(ring_position, dtype=float).reshape(3)
    )
    rotation_world_from_ring = (
        np.eye(3, dtype=float)
        if rotation_world_from_ring is None
        else np.asarray(rotation_world_from_ring, dtype=float).reshape(3, 3)
    )

    # Deliberately compute the expected frame independently of the new production
    # helpers so this test can catch a shared sign/frame bug.
    theta = geometry.angle_zero_rad + 2.0 * math.pi * plate_index / geometry.plate_count
    rotation_ring_from_plate = _rz(theta)
    position_world = ring_position + rotation_world_from_ring @ geometry.plate_body_offset(plate_index)
    rotation_world_from_plate = rotation_world_from_ring @ rotation_ring_from_plate
    return position_world, rotation_world_from_plate


def _test_config(api):
    # 15 deg outward from +z_P.  This is only a deterministic unit-test vector,
    # not a locked M2 hold/proof angle.
    angle = math.radians(15.0)
    return api.AttachmentDetectorConfig(
        candidate_xy_m=0.020,
        candidate_normal_m=0.015,
        candidate_speed_mps=0.30,
        candidate_dwell_s=0.10,
        proof_xy_m=0.025,
        proof_normal_m=0.020,
        proof_speed_mps=0.35,
        proof_dwell_s=0.10,
        proof_min_excitation_m=0.030,
        proof_direction_plate=np.array([math.sin(angle), 0.0, math.cos(angle)]),
        loss_xy_m=0.050,
        loss_normal_m=0.040,
        loss_dwell_s=0.15,
        pose_timeout_s=0.10,
        velocity_filter_tau_s=0.04,
    )


def _calibration(api, *, contact_offset_m=(0.0, 0.0, 0.0), rotation=None):
    if rotation is None:
        rotation = np.eye(3)
    return api.MagnetContactCalibration(
        contact_position_magnet=np.asarray(contact_offset_m, dtype=float),
        rotation_magnet_from_contact=np.asarray(rotation, dtype=float),
    )


def _detector(api, *, plate_index=0, calibration=None):
    if calibration is None:
        calibration = _calibration(api)
    return api.AttachmentDetector(
        geometry=RingNetGeometry(),
        assigned_plate_id=plate_index,
        calibration=calibration,
        config=_test_config(api),
    )


def _observation(
    api,
    *,
    t: float,
    plate_index: int = 0,
    ring_position=None,
    rotation_world_from_ring=None,
    magnet_position_world=None,
    rotation_world_from_magnet=None,
    drone_position_world=None,
    magnet_on: bool = True,
    proof_requested: bool = False,
    ring_time_s: float | None = None,
    magnet_time_s: float | None = None,
    drone_time_s: float | None = None,
):
    geometry = RingNetGeometry()
    ring_position = np.zeros(3) if ring_position is None else np.asarray(ring_position, dtype=float)
    rotation_world_from_ring = (
        np.eye(3)
        if rotation_world_from_ring is None
        else np.asarray(rotation_world_from_ring, dtype=float)
    )
    plate_position, plate_rotation = _plate_pose(
        geometry,
        plate_index,
        ring_position=ring_position,
        rotation_world_from_ring=rotation_world_from_ring,
    )
    if magnet_position_world is None:
        magnet_position_world = plate_position.copy()
    if rotation_world_from_magnet is None:
        rotation_world_from_magnet = plate_rotation.copy()
    if drone_position_world is None:
        drone_position_world = plate_position + plate_rotation @ np.array([0.0, 0.0, 0.50])

    stamp = float(t)
    return api.AttachmentObservation(
        time_s=stamp,
        ring_position_world=ring_position,
        rotation_world_from_ring=rotation_world_from_ring,
        ring_time_s=stamp if ring_time_s is None else float(ring_time_s),
        magnet_position_world=np.asarray(magnet_position_world, dtype=float),
        rotation_world_from_magnet=np.asarray(rotation_world_from_magnet, dtype=float),
        magnet_time_s=stamp if magnet_time_s is None else float(magnet_time_s),
        drone_position_world=np.asarray(drone_position_world, dtype=float),
        drone_time_s=stamp if drone_time_s is None else float(drone_time_s),
        magnet_on=bool(magnet_on),
        proof_requested=bool(proof_requested),
    )


def _state_value(output) -> str:
    return output.state.value


def _advance_candidate(api, detector, *, plate_index=0, start_t=0.0):
    output = None
    for dt in (0.0, 0.05, 0.11):
        output = detector.update(_observation(api, t=start_t + dt, plate_index=plate_index))
    assert output is not None
    return output


def test_attachment_detection_module_is_ros_and_gazebo_independent() -> None:
    module_path = (
        Path(__file__).resolve().parents[1]
        / "tejen_mission"
        / "attachment_detection.py"
    )
    assert module_path.is_file(), (
        "M2A test-first RED: attachment_detection.py must be added as a pure "
        "ROS/Gazebo-independent module."
    )

    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    forbidden_prefixes = (
        "rclpy",
        "geometry_msgs",
        "std_msgs",
        "sensor_msgs",
        "nav_msgs",
        "gazebo",
        "gz",
        "ros_gz",
    )
    imported = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.append(node.module)

    forbidden = [name for name in imported if name.startswith(forbidden_prefixes)]
    assert forbidden == [], f"attachment_detection.py must stay pure; forbidden imports: {forbidden}"


def test_gazebo_joint_truth_is_not_part_of_detector_observation_api() -> None:
    api = _api()
    observation_fields = {field.name for field in fields(api.AttachmentObservation)}
    config_fields = {field.name for field in fields(api.AttachmentDetectorConfig)}

    assert "gazebo_joint_truth" not in observation_fields
    assert "joint_truth" not in observation_fields
    assert "gazebo_joint_truth" not in config_fields
    assert "joint_truth" not in config_fields


def test_contact_frame_calibration_places_contact_surface_on_plate() -> None:
    api = _api()
    geometry = RingNetGeometry()
    plate_index = 5
    plate_position, plate_rotation = _plate_pose(geometry, plate_index)

    # Mocap/magnet rigid-body origin is 5 cm above the physical contact face.
    calibration = _calibration(api, contact_offset_m=(0.0, 0.0, -0.05))
    detector = _detector(api, plate_index=plate_index, calibration=calibration)

    magnet_origin = plate_position + plate_rotation @ np.array([0.0, 0.0, 0.05])
    output = detector.update(
        _observation(
            api,
            t=0.0,
            plate_index=plate_index,
            magnet_position_world=magnet_origin,
            rotation_world_from_magnet=plate_rotation,
        )
    )

    np.testing.assert_allclose(output.plate_relative_position_m, np.zeros(3), atol=1e-12)
    assert output.radial_error_m == pytest.approx(0.0, abs=1e-12)
    assert output.tangential_error_m == pytest.approx(0.0, abs=1e-12)
    assert output.normal_error_m == pytest.approx(0.0, abs=1e-12)
    assert output.xy_error_m == pytest.approx(0.0, abs=1e-12)
    assert output.orientation_error_rad == pytest.approx(0.0, abs=1e-12)


def test_relative_orientation_is_diagnostic_but_not_required_for_candidate() -> None:
    api = _api()
    detector = _detector(api)
    _, plate_rotation = _plate_pose(RingNetGeometry(), 0)

    output = detector.update(
        _observation(
            api,
            t=0.0,
            rotation_world_from_magnet=plate_rotation @ _rx(math.radians(20.0)),
        )
    )

    assert output.orientation_error_rad == pytest.approx(math.radians(20.0), abs=1e-9)
    assert output.candidate_condition is True


def test_same_plate_relative_contact_is_invariant_across_assigned_plate_ids() -> None:
    api = _api()

    for plate_index in (0, 1, 5, 11):
        detector = _detector(api, plate_index=plate_index)
        output = detector.update(_observation(api, t=0.0, plate_index=plate_index))
        np.testing.assert_allclose(output.plate_relative_position_m, np.zeros(3), atol=1e-12)
        assert output.xy_error_m == pytest.approx(0.0, abs=1e-12)
        assert output.normal_error_m == pytest.approx(0.0, abs=1e-12)


def test_candidate_requires_continuous_dwell_and_resets_when_bounds_are_left() -> None:
    api = _api()
    detector = _detector(api)

    out0 = detector.update(_observation(api, t=0.00))
    out1 = detector.update(_observation(api, t=0.06))
    assert _state_value(out0) == "contact_candidate"
    assert _state_value(out1) == "contact_candidate"
    assert out1.candidate_dwell_s == pytest.approx(0.06, abs=1e-9)

    plate_position, _ = _plate_pose(RingNetGeometry(), 0)
    outside = detector.update(
        _observation(
            api,
            t=0.07,
            magnet_position_world=plate_position + np.array([0.030, 0.0, 0.0]),
        )
    )
    assert outside.candidate_condition is False
    assert outside.candidate_dwell_s == pytest.approx(0.0)

    restarted = detector.update(_observation(api, t=0.08))
    assert restarted.candidate_dwell_s == pytest.approx(0.0)
    later = detector.update(_observation(api, t=0.19))
    assert later.candidate_dwell_s >= 0.10
    assert _state_value(later) == "contact_candidate"


def test_candidate_requires_magnet_on() -> None:
    api = _api()
    detector = _detector(api)
    output = detector.update(_observation(api, t=0.0, magnet_on=False))
    assert output.candidate_condition is False
    assert _state_value(output) == "separated"


def test_stale_pose_input_produces_unavailable_not_attachment_evidence() -> None:
    api = _api()
    detector = _detector(api)

    output = detector.update(
        _observation(
            api,
            t=1.0,
            ring_time_s=1.0,
            magnet_time_s=0.80,
            drone_time_s=1.0,
        )
    )
    assert output.fresh is False
    assert _state_value(output) == "unavailable"
    assert output.confirmed is False


def test_proof_requested_without_measured_excitation_cannot_confirm() -> None:
    api = _api()
    detector = _detector(api)
    _advance_candidate(api, detector)

    output = None
    for t in np.arange(0.12, 0.50, 0.03):
        output = detector.update(
            _observation(api, t=float(t), proof_requested=True)
        )
    assert output is not None
    assert _state_value(output) == "proving"
    assert output.proof_excitation_m == pytest.approx(0.0, abs=1e-12)
    assert output.confirmed is False


def test_attached_magnet_confirms_after_real_proof_excitation_and_dwell() -> None:
    api = _api()
    detector = _detector(api)
    _advance_candidate(api, detector)

    geometry = RingNetGeometry()
    plate_position, plate_rotation = _plate_pose(geometry, 0)
    proof_direction = _test_config(api).proof_direction_plate
    base_drone_plate = np.array([0.0, 0.0, 0.50])

    output = None
    for step, t in enumerate(np.arange(0.12, 0.45, 0.03)):
        progress = min(0.045, 0.006 * step)
        drone_world = plate_position + plate_rotation @ (
            base_drone_plate + progress * proof_direction
        )
        output = detector.update(
            _observation(
                api,
                t=float(t),
                drone_position_world=drone_world,
                proof_requested=True,
            )
        )

    assert output is not None
    assert output.proof_excitation_m >= 0.030
    assert output.confirmed is True
    assert _state_value(output) == "confirmed"


def test_irl_magnet_off_preserves_confirmation_until_geometry_proves_separation() -> None:
    api = _api()
    config = replace(_test_config(api), magnet_off_counts_as_loss=False)
    detector = api.AttachmentDetector(
        geometry=RingNetGeometry(),
        assigned_plate_id=0,
        calibration=_calibration(api),
        config=config,
    )
    _advance_candidate(api, detector)
    geometry = RingNetGeometry()
    plate_position, plate_rotation = _plate_pose(geometry, 0)
    proof_direction = config.proof_direction_plate
    base_drone_plate = np.array([0.0, 0.0, 0.50])
    output = None
    for step, t in enumerate(np.arange(0.12, 0.45, 0.03)):
        progress = min(0.045, 0.006 * step)
        output = detector.update(
            _observation(
                api,
                t=float(t),
                drone_position_world=plate_position
                + plate_rotation @ (base_drone_plate + progress * proof_direction),
                proof_requested=True,
            )
        )
    assert output is not None and output.confirmed

    for t in (0.50, 0.60, 0.70):
        output = detector.update(_observation(api, t=t, magnet_on=False))
    assert output.confirmed
    assert not output.lost
    assert not output.geometry_separated

    separated = plate_position + plate_rotation @ np.array([0.08, 0.0, 0.0])
    for t in (0.80, 0.90, 1.00):
        output = detector.update(
            _observation(
                api,
                t=t,
                magnet_on=False,
                magnet_position_world=separated,
            )
        )
    assert output.geometry_separated
    assert output.lost
    assert not output.confirmed


def test_unattached_magnet_that_is_pulled_away_during_proof_never_confirms() -> None:
    api = _api()
    detector = _detector(api)
    _advance_candidate(api, detector)

    geometry = RingNetGeometry()
    plate_position, plate_rotation = _plate_pose(geometry, 0)
    proof_direction = _test_config(api).proof_direction_plate
    base_drone_plate = np.array([0.0, 0.0, 0.50])

    output = None
    for step, t in enumerate(np.arange(0.12, 0.45, 0.03)):
        progress = min(0.050, 0.007 * step)
        drone_world = plate_position + plate_rotation @ (
            base_drone_plate + progress * proof_direction
        )
        # Unattached magnet is dragged away from the plate as the cable is loaded.
        magnet_world = plate_position + plate_rotation @ (
            1.4 * progress * proof_direction
        )
        output = detector.update(
            _observation(
                api,
                t=float(t),
                magnet_position_world=magnet_world,
                drone_position_world=drone_world,
                proof_requested=True,
            )
        )

    assert output is not None
    assert output.proof_excitation_m >= 0.030
    assert output.confirmed is False
    assert _state_value(output) != "confirmed"


def test_fixed_seed_sensor_noise_does_not_prevent_valid_attachment_confirmation() -> None:
    api = _api()
    detector = _detector(api)
    geometry = RingNetGeometry()
    plate_position, plate_rotation = _plate_pose(geometry, 0)
    proof_direction = _test_config(api).proof_direction_plate
    base_drone_plate = np.array([0.0, 0.0, 0.50])
    rng = np.random.default_rng(20260909)

    output = None
    for k in range(30):
        t = k / 30.0
        progress = 0.0 if k < 7 else min(0.045, 0.0035 * (k - 6))
        proof_requested = k >= 7

        # 0.5 mm position noise is intentionally small but nonzero.  The test is
        # deterministic and exercises the real finite-difference/filter path.
        magnet_noise = rng.normal(0.0, 0.0005, size=3)
        drone_noise = rng.normal(0.0, 0.0005, size=3)
        magnet_world = plate_position + magnet_noise
        drone_world = plate_position + plate_rotation @ (
            base_drone_plate + progress * proof_direction
        ) + drone_noise

        output = detector.update(
            _observation(
                api,
                t=t,
                magnet_position_world=magnet_world,
                drone_position_world=drone_world,
                proof_requested=proof_requested,
            )
        )

    assert output is not None
    assert output.confirmed is True
    assert _state_value(output) == "confirmed"


def test_moving_yawing_ring_keeps_attached_magnet_near_zero_in_plate_frame() -> None:
    api = _api()
    plate_index = 3
    detector = _detector(api, plate_index=plate_index)
    geometry = RingNetGeometry()

    outputs = []
    for k in range(12):
        t = k / 30.0
        ring_position = np.array([0.10 * t, -0.05 * t, 1.0])
        rotation_world_from_ring = _rz(0.25 * t)
        plate_position, plate_rotation = _plate_pose(
            geometry,
            plate_index,
            ring_position=ring_position,
            rotation_world_from_ring=rotation_world_from_ring,
        )
        drone_world = plate_position + plate_rotation @ np.array([0.0, 0.0, 0.50])
        outputs.append(
            detector.update(
                _observation(
                    api,
                    t=t,
                    plate_index=plate_index,
                    ring_position=ring_position,
                    rotation_world_from_ring=rotation_world_from_ring,
                    magnet_position_world=plate_position,
                    rotation_world_from_magnet=plate_rotation,
                    drone_position_world=drone_world,
                )
            )
        )

    for output in outputs:
        assert output.xy_error_m == pytest.approx(0.0, abs=1e-9)
        assert output.normal_error_m == pytest.approx(0.0, abs=1e-9)
    # Ignore the first sample because finite-difference velocity has no history yet.
    for output in outputs[1:]:
        assert output.relative_speed_mps == pytest.approx(0.0, abs=1e-8)


def test_confirmed_state_uses_hysteresis_single_spike_does_not_mark_lost() -> None:
    api = _api()
    detector = _detector(api)
    _advance_candidate(api, detector)

    geometry = RingNetGeometry()
    plate_position, plate_rotation = _plate_pose(geometry, 0)
    proof_direction = _test_config(api).proof_direction_plate
    base_drone_plate = np.array([0.0, 0.0, 0.50])

    # Confirm first.
    output = None
    for step, t in enumerate(np.arange(0.12, 0.45, 0.03)):
        progress = min(0.045, 0.006 * step)
        output = detector.update(
            _observation(
                api,
                t=float(t),
                drone_position_world=plate_position
                + plate_rotation @ (base_drone_plate + progress * proof_direction),
                proof_requested=True,
            )
        )
    assert output is not None and output.confirmed

    # One large 30 ms excursion is shorter than loss_dwell_s.
    spike = detector.update(
        _observation(
            api,
            t=0.48,
            magnet_position_world=plate_position + np.array([0.070, 0.0, 0.0]),
            drone_position_world=plate_position + plate_rotation @ (base_drone_plate + 0.045 * proof_direction),
            proof_requested=True,
        )
    )
    assert spike.confirmed is True
    assert _state_value(spike) == "confirmed"

    recovered = detector.update(
        _observation(
            api,
            t=0.51,
            drone_position_world=plate_position + plate_rotation @ (base_drone_plate + 0.045 * proof_direction),
            proof_requested=True,
        )
    )
    assert recovered.confirmed is True
    assert _state_value(recovered) == "confirmed"


def test_sustained_separation_after_confirmation_transitions_to_lost() -> None:
    api = _api()
    detector = _detector(api)
    _advance_candidate(api, detector)

    geometry = RingNetGeometry()
    plate_position, plate_rotation = _plate_pose(geometry, 0)
    proof_direction = _test_config(api).proof_direction_plate
    base_drone_plate = np.array([0.0, 0.0, 0.50])

    output = None
    for step, t in enumerate(np.arange(0.12, 0.45, 0.03)):
        progress = min(0.045, 0.006 * step)
        output = detector.update(
            _observation(
                api,
                t=float(t),
                drone_position_world=plate_position
                + plate_rotation @ (base_drone_plate + progress * proof_direction),
                proof_requested=True,
            )
        )
    assert output is not None and output.confirmed

    lost = None
    for t in np.arange(0.48, 0.72, 0.03):
        lost = detector.update(
            _observation(
                api,
                t=float(t),
                magnet_position_world=plate_position + np.array([0.070, 0.0, 0.0]),
                drone_position_world=plate_position
                + plate_rotation @ (base_drone_plate + 0.045 * proof_direction),
                proof_requested=True,
            )
        )
    assert lost is not None
    assert lost.confirmed is False
    assert lost.lost is True
    assert _state_value(lost) == "lost"


def test_candidate_readiness_is_revoked_if_contact_is_lost_before_proof() -> None:
    """A completed candidate dwell is not a permanent licence to start proof."""

    api = _api()
    detector = _detector(api)
    _advance_candidate(api, detector)

    plate_position, _ = _plate_pose(RingNetGeometry(), 0)
    # Outside the candidate bound (20 mm) but still inside the proof bound
    # (25 mm).  This catches a stale candidate-ready latch specifically.
    outside_candidate = plate_position + np.array([0.023, 0.0, 0.0])
    lost_contact = detector.update(
        _observation(
            api,
            t=0.12,
            magnet_position_world=outside_candidate,
            proof_requested=False,
        )
    )
    assert lost_contact.candidate_condition is False

    attempted_proof = detector.update(
        _observation(
            api,
            t=0.15,
            magnet_position_world=outside_candidate,
            proof_requested=True,
        )
    )
    assert _state_value(attempted_proof) == "separated"
    assert attempted_proof.confirmed is False


def test_stale_sample_does_not_contaminate_next_fresh_relative_velocity() -> None:
    """Offline replay may contain stale/dropout rows; they must not poison filtering."""

    api = _api()
    detector = _detector(api)
    plate_position, _ = _plate_pose(RingNetGeometry(), 0)

    first = detector.update(_observation(api, t=0.0))
    assert first.relative_speed_mps == pytest.approx(0.0)

    stale = detector.update(
        _observation(
            api,
            t=0.20,
            magnet_position_world=plate_position + np.array([0.10, 0.0, 0.0]),
            magnet_time_s=0.0,
        )
    )
    assert _state_value(stale) == "unavailable"

    recovered = detector.update(_observation(api, t=0.21))
    assert recovered.fresh is True
    assert recovered.relative_speed_mps == pytest.approx(0.0, abs=1e-12)
    assert recovered.candidate_condition is True


def test_stale_after_confirmation_reports_no_current_confirmation_evidence() -> None:
    """Internal hysteresis may be retained, but stale output must not authorize progression."""

    api = _api()
    detector = _detector(api)
    _advance_candidate(api, detector)

    geometry = RingNetGeometry()
    plate_position, plate_rotation = _plate_pose(geometry, 0)
    proof_direction = _test_config(api).proof_direction_plate
    base_drone_plate = np.array([0.0, 0.0, 0.50])

    output = None
    for step, t in enumerate(np.arange(0.12, 0.45, 0.03)):
        progress = min(0.045, 0.006 * step)
        output = detector.update(
            _observation(
                api,
                t=float(t),
                drone_position_world=plate_position
                + plate_rotation @ (base_drone_plate + progress * proof_direction),
                proof_requested=True,
            )
        )
    assert output is not None and output.confirmed

    stale = detector.update(
        _observation(
            api,
            t=0.60,
            magnet_time_s=0.40,
            drone_position_world=plate_position
            + plate_rotation @ (base_drone_plate + 0.045 * proof_direction),
            proof_requested=True,
        )
    )
    assert _state_value(stale) == "unavailable"
    assert stale.confirmed is False


def test_stale_gap_before_proof_invalidates_candidate_readiness() -> None:
    """A stale observation gap must force contact evidence to be reacquired."""

    api = _api()
    detector = _detector(api)
    _advance_candidate(api, detector)

    stale = detector.update(
        _observation(
            api,
            t=0.30,
            magnet_time_s=0.10,
            proof_requested=False,
        )
    )
    assert _state_value(stale) == "unavailable"

    attempted_proof = detector.update(
        _observation(api, t=0.31, proof_requested=True)
    )
    assert _state_value(attempted_proof) != "proving"
    assert attempted_proof.confirmed is False
