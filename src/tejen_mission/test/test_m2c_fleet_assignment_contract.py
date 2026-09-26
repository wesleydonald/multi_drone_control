"""Tests-first behavioural contract for M2C fleet assignment.

These tests intentionally describe the public, ROS-independent contract before the
fleet implementation exists.  They should be RED on the post-M2B baseline and remain
useful after M2C implementation.  The tests constrain mission behaviour, not the
internal algorithm beyond the already-agreed exhaustive 72-candidate search.
"""

from __future__ import annotations

import importlib
import math

import numpy as np
import pytest

from tejen_mission.cooperative_trajectory import RingNetGeometry
from tejen_mission.m2b_attachment_mission import (
    M2BConfig,
    attached_hold_direction_plate,
    body_reference_from_contact,
)


VEHICLE_IDS = ("drone_0", "drone_1", "drone_2", "drone_3")
PHYSICAL_IDS = (0, 1, 2, 3)
TETHER_LENGTH_M = 0.475
TETHER_ANCHOR_BODY = np.array([0.0, 0.0, -0.04], dtype=float)
ATTACHED_HOLD_ANGLE_DEG = 15.0
STATE_TIMEOUT_S = 0.25


def _fleet_module():
    try:
        return importlib.import_module("tejen_mission.m2_fleet_assignment")
    except ModuleNotFoundError as exc:
        pytest.fail(
            "M2C RED contract: tejen_mission.m2_fleet_assignment does not exist yet"
        )


def _yaw_rotation(yaw_rad: float) -> np.ndarray:
    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _required_symbol(module, name: str):
    value = getattr(module, name, None)
    assert value is not None, f"M2C RED contract: missing public symbol {name}"
    return value


def _config(module):
    cls = _required_symbol(module, "FleetAssignmentConfig")
    return cls(
        state_timeout_s=STATE_TIMEOUT_S,
        tether_length_m=TETHER_LENGTH_M,
        tether_anchor_body=TETHER_ANCHOR_BODY,
        attached_hold_angle_deg=ATTACHED_HOLD_ANGLE_DEG,
    )


def _vehicle(module, physical_id: int, position, *, yaw_rad=0.0, stamp_s=10.0, vehicle_id=None):
    cls = _required_symbol(module, "FleetVehicleState")
    physical_id = int(physical_id)
    return cls(
        vehicle_id=VEHICLE_IDS[physical_id] if vehicle_id is None else str(vehicle_id),
        physical_drone_id=physical_id,
        namespace=f"/drone_{physical_id}",
        mocap_rigid_body_id=100 + physical_id,
        position_world=np.asarray(position, dtype=float),
        yaw_rad=float(yaw_rad),
        stamp_s=float(stamp_s),
    )


def _candidate_mapping(candidate) -> dict[str, int]:
    mapping = getattr(candidate, "vehicle_to_plate", None)
    assert mapping is not None, "M2C assignment candidate must expose vehicle_to_plate"
    return {str(k): int(v) for k, v in dict(mapping).items()}


def _expected_hold_target(
    *,
    geometry: RingNetGeometry,
    ring_position: np.ndarray,
    rotation_world_from_ring: np.ndarray,
    plate_index: int,
    vehicle_yaw_rad: float,
) -> np.ndarray:
    cfg = M2BConfig(attached_hold_angle_deg=ATTACHED_HOLD_ANGLE_DEG)
    plate_position = geometry.plate_position_world(
        ring_position=ring_position,
        rotation_world_from_ring=rotation_world_from_ring,
        plate_index=plate_index,
    )
    plate_rotation = geometry.plate_rotation_world(
        rotation_world_from_ring=rotation_world_from_ring,
        plate_index=plate_index,
    )
    direction_world = plate_rotation @ attached_hold_direction_plate(cfg)
    return body_reference_from_contact(
        contact_position_world=plate_position,
        direction_contact_to_vehicle_world=direction_world,
        tether_length_m=TETHER_LENGTH_M,
        rotation_world_from_body=_yaw_rotation(vehicle_yaw_rad),
        tether_anchor_body=TETHER_ANCHOR_BODY,
    )


def test_m2c_enumerates_exactly_72_symmetric_assignment_candidates():
    module = _fleet_module()
    enumerate_candidates = _required_symbol(module, "enumerate_candidate_assignments")
    candidates = tuple(enumerate_candidates(VEHICLE_IDS))

    assert len(candidates) == 72
    observed_sets = set()
    for candidate in candidates:
        mapping = _candidate_mapping(candidate)
        assert set(mapping) == set(VEHICLE_IDS)
        observed_sets.add(tuple(sorted(mapping.values())))

    assert observed_sets == {
        (0, 3, 6, 9),
        (1, 4, 7, 10),
        (2, 5, 8, 11),
    }
    for candidate in candidates:
        mapping = _candidate_mapping(candidate)
        assert len(mapping) == 4
        assert len(set(mapping.values())) == 4
        assert all(0 <= plate_id < 12 for plate_id in mapping.values())

    # The contract must permit a valid assignment in which every physical vehicle
    # goes to a plate whose number differs from its hardware index.
    assert any(
        all(mapping[f"drone_{i}"] != i for i in PHYSICAL_IDS)
        for mapping in map(_candidate_mapping, candidates)
    )


def test_m2c_shared_attached_hold_target_matches_existing_b1_geometry_at_20deg_ring_yaw():
    module = _fleet_module()
    target_fn = _required_symbol(module, "attached_hold_body_target_world")
    geometry = RingNetGeometry()
    ring_position = np.array([0.35, -0.20, 0.035], dtype=float)
    ring_rotation = _yaw_rotation(math.radians(20.0))
    vehicle_yaw = math.radians(-31.0)
    plate_id = 7

    actual = np.asarray(
        target_fn(
            geometry=geometry,
            ring_position_world=ring_position,
            rotation_world_from_ring=ring_rotation,
            plate_index=plate_id,
            vehicle_yaw_rad=vehicle_yaw,
            tether_length_m=TETHER_LENGTH_M,
            tether_anchor_body=TETHER_ANCHOR_BODY,
            attached_hold_angle_deg=ATTACHED_HOLD_ANGLE_DEG,
        ),
        dtype=float,
    )
    expected = _expected_hold_target(
        geometry=geometry,
        ring_position=ring_position,
        rotation_world_from_ring=ring_rotation,
        plate_index=plate_id,
        vehicle_yaw_rad=vehicle_yaw,
    )
    np.testing.assert_allclose(actual, expected, atol=1e-10, rtol=0.0)


def test_m2c_asymmetric_hold_geometry_selects_the_known_unique_assignment():
    module = _fleet_module()
    choose = _required_symbol(module, "choose_fleet_assignment")
    geometry = RingNetGeometry()
    ring_position = np.array([0.10, -0.15, 0.035], dtype=float)
    ring_rotation = _yaw_rotation(math.radians(20.0))
    desired = {
        "drone_0": 7,
        "drone_1": 1,
        "drone_2": 10,
        "drone_3": 4,
    }
    yaws = [math.radians(v) for v in (12.0, -18.0, 33.0, -41.0)]
    vehicles = []
    for i, vehicle_id in enumerate(VEHICLE_IDS):
        target = _expected_hold_target(
            geometry=geometry,
            ring_position=ring_position,
            rotation_world_from_ring=ring_rotation,
            plate_index=desired[vehicle_id],
            vehicle_yaw_rad=yaws[i],
        )
        vehicles.append(_vehicle(module, i, target, yaw_rad=yaws[i]))

    assignment = choose(
        vehicles=vehicles,
        ring_position_world=ring_position,
        rotation_world_from_ring=ring_rotation,
        now_s=10.10,
        geometry=geometry,
        config=_config(module),
    )
    assert _candidate_mapping(assignment) == desired
    assert int(getattr(assignment, "candidate_count")) == 72
    assert float(getattr(assignment, "total_cost")) == pytest.approx(0.0, abs=1e-12)


def test_m2c_equal_cost_ties_are_deterministic_set0_then_lexicographic():
    module = _fleet_module()
    choose = _required_symbol(module, "choose_fleet_assignment")
    geometry = RingNetGeometry()
    ring_position = np.array([0.0, 0.0, 0.035], dtype=float)
    ring_rotation = _yaw_rotation(math.radians(20.0))

    # Coincident synthetic start positions intentionally make all symmetric plate
    # assignments equal-cost.  Deterministic replay must select set 0 and the
    # lexicographically smallest vehicle->plate mapping.
    vehicles = [_vehicle(module, i, ring_position, stamp_s=20.0) for i in PHYSICAL_IDS]
    first = choose(
        vehicles=vehicles,
        ring_position_world=ring_position,
        rotation_world_from_ring=ring_rotation,
        now_s=20.05,
        geometry=geometry,
        config=_config(module),
    )
    second = choose(
        vehicles=list(reversed(vehicles)),
        ring_position_world=ring_position,
        rotation_world_from_ring=ring_rotation,
        now_s=20.05,
        geometry=geometry,
        config=_config(module),
    )

    expected = {"drone_0": 0, "drone_1": 3, "drone_2": 6, "drone_3": 9}
    assert _candidate_mapping(first) == expected
    assert _candidate_mapping(second) == expected
    assert int(getattr(first, "plate_set_index")) == 0
    assert int(getattr(second, "plate_set_index")) == 0


@pytest.mark.parametrize("failure_kind", ["missing", "duplicate_vehicle", "stale"])
def test_m2c_assignment_fails_closed_on_incomplete_duplicate_or_stale_fleet_state(failure_kind):
    module = _fleet_module()
    choose = _required_symbol(module, "choose_fleet_assignment")
    geometry = RingNetGeometry()
    ring_position = np.array([0.0, 0.0, 0.035], dtype=float)
    ring_rotation = np.eye(3)
    vehicles = [
        _vehicle(module, i, [0.8 * math.cos(i), 0.8 * math.sin(i), 0.105], stamp_s=30.0)
        for i in PHYSICAL_IDS
    ]

    if failure_kind == "missing":
        vehicles = vehicles[:-1]
    elif failure_kind == "duplicate_vehicle":
        vehicles[-1] = _vehicle(
            module, 3, vehicles[-1].position_world, stamp_s=30.0, vehicle_id="drone_0"
        )
    elif failure_kind == "stale":
        vehicles[-1] = _vehicle(module, 3, vehicles[-1].position_world, stamp_s=29.0)

    with pytest.raises((ValueError, RuntimeError)):
        choose(
            vehicles=vehicles,
            ring_position_world=ring_position,
            rotation_world_from_ring=ring_rotation,
            now_s=30.10,
            geometry=geometry,
            config=_config(module),
        )


def test_m2c_frozen_assignment_cannot_silently_reassign_after_retry_or_new_measurement():
    module = _fleet_module()
    latch_cls = _required_symbol(module, "FleetAssignmentLatch")
    enumerate_candidates = _required_symbol(module, "enumerate_candidate_assignments")
    candidates = tuple(enumerate_candidates(VEHICLE_IDS))
    assert len(candidates) >= 2

    latch = latch_cls()
    first = candidates[0]
    other = next(c for c in candidates[1:] if _candidate_mapping(c) != _candidate_mapping(first))
    frozen = latch.freeze(first)
    assert _candidate_mapping(frozen) == _candidate_mapping(first)
    assert _candidate_mapping(latch.assignment) == _candidate_mapping(first)

    with pytest.raises((ValueError, RuntimeError)):
        latch.freeze(other)
    assert _candidate_mapping(latch.assignment) == _candidate_mapping(first)
