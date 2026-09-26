"""Pure M2C fleet identity and frozen plate-assignment logic.

M2 deliberately has only one non-stationary mission vehicle at a time.  The
assignment problem is therefore kept small and transparent: evaluate the three
symmetric four-plate sets and all 24 permutations in each set, then minimize
summed squared distance from measured start positions to the B1-compatible
attached-hold body targets.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import itertools
import math
from types import MappingProxyType
from typing import Iterable, Mapping, Sequence

import numpy as np

from .cooperative_trajectory import RingNetGeometry
from .m2b_attachment_mission import (
    M2BConfig,
    attached_hold_direction_plate,
    body_reference_from_contact,
)


SYMMETRIC_PLATE_SETS: tuple[tuple[int, int, int, int], ...] = (
    (0, 3, 6, 9),
    (1, 4, 7, 10),
    (2, 5, 8, 11),
)

OPPOSITE_PLATE_PAIRS: tuple[tuple[int, int], ...] = tuple(
    (plate, plate + 6) for plate in range(6)
)


def _vec3(value, *, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite")
    return result.copy()


def _rotation(value, *, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be finite")
    if abs(float(np.linalg.det(result)) - 1.0) > 1e-5:
        raise ValueError(f"{name} must be a proper rotation")
    if not np.allclose(result.T @ result, np.eye(3), atol=1e-5, rtol=0.0):
        raise ValueError(f"{name} must be orthonormal")
    return result.copy()


def _yaw_rotation(yaw_rad: float) -> np.ndarray:
    yaw = float(yaw_rad)
    if not math.isfinite(yaw):
        raise ValueError("vehicle yaw must be finite")
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


@dataclass(frozen=True)
class FleetAssignmentConfig:
    state_timeout_s: float = 0.25
    ring_state_timeout_s: float = 0.25
    tether_length_m: float = 0.475
    tether_anchor_body: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, -0.04], dtype=float)
    )
    attached_hold_angle_deg: float = 15.0

    def __post_init__(self) -> None:
        state_timeout = float(self.state_timeout_s)
        ring_timeout = float(self.ring_state_timeout_s)
        tether_length = float(self.tether_length_m)
        angle = float(self.attached_hold_angle_deg)
        if not math.isfinite(state_timeout) or state_timeout <= 0.0:
            raise ValueError("state_timeout_s must be finite and positive")
        if not math.isfinite(ring_timeout) or ring_timeout <= 0.0:
            raise ValueError("ring_state_timeout_s must be finite and positive")
        if not math.isfinite(tether_length) or tether_length <= 0.0:
            raise ValueError("tether_length_m must be finite and positive")
        if not math.isfinite(angle):
            raise ValueError("attached_hold_angle_deg must be finite")
        object.__setattr__(self, "state_timeout_s", state_timeout)
        object.__setattr__(self, "ring_state_timeout_s", ring_timeout)
        object.__setattr__(self, "tether_length_m", tether_length)
        object.__setattr__(self, "attached_hold_angle_deg", angle)
        object.__setattr__(self, "tether_anchor_body", _vec3(self.tether_anchor_body, name="tether_anchor_body"))


@dataclass(frozen=True)
class FleetVehicleState:
    vehicle_id: str
    physical_drone_id: int
    namespace: str
    mocap_rigid_body_id: int
    position_world: np.ndarray
    yaw_rad: float
    stamp_s: float

    def __post_init__(self) -> None:
        vehicle_id = str(self.vehicle_id).strip()
        namespace = str(self.namespace).strip()
        physical = int(self.physical_drone_id)
        rigid_body = int(self.mocap_rigid_body_id)
        yaw = float(self.yaw_rad)
        stamp = float(self.stamp_s)
        if not vehicle_id:
            raise ValueError("vehicle_id must be non-empty")
        if physical < 0 or rigid_body < 0:
            raise ValueError("physical/mocap IDs must be non-negative")
        if not namespace.startswith("/drone_"):
            raise ValueError("namespace must follow /drone_<id> convention")
        if not math.isfinite(yaw) or not math.isfinite(stamp):
            raise ValueError("yaw/stamp must be finite")
        object.__setattr__(self, "vehicle_id", vehicle_id)
        object.__setattr__(self, "namespace", namespace)
        object.__setattr__(self, "physical_drone_id", physical)
        object.__setattr__(self, "mocap_rigid_body_id", rigid_body)
        object.__setattr__(self, "position_world", _vec3(self.position_world, name="position_world"))
        object.__setattr__(self, "yaw_rad", yaw)
        object.__setattr__(self, "stamp_s", stamp)


@dataclass(frozen=True)
class FleetAssignment:
    vehicle_to_plate: Mapping[str, int]
    plate_set_index: int
    total_cost: float = math.inf
    candidate_count: int = 72

    def __post_init__(self) -> None:
        mapping = {str(key): int(value) for key, value in dict(self.vehicle_to_plate).items()}
        if len(mapping) != 4 or len(set(mapping.values())) != 4:
            raise ValueError("fleet assignment must contain four unique vehicle/plate bindings")
        if int(self.plate_set_index) not in range(len(SYMMETRIC_PLATE_SETS)):
            raise ValueError("plate_set_index out of range")
        expected_set = set(SYMMETRIC_PLATE_SETS[int(self.plate_set_index)])
        if set(mapping.values()) != expected_set:
            raise ValueError("assignment plates do not match selected symmetric set")
        cost = float(self.total_cost)
        if not (math.isfinite(cost) or math.isinf(cost)) or cost < 0.0:
            raise ValueError("total_cost must be non-negative")
        object.__setattr__(self, "vehicle_to_plate", MappingProxyType(mapping))
        object.__setattr__(self, "plate_set_index", int(self.plate_set_index))
        object.__setattr__(self, "total_cost", cost)
        object.__setattr__(self, "candidate_count", int(self.candidate_count))


class FleetAssignmentLatch:
    """One-way assignment freeze for one mission epoch."""

    def __init__(self) -> None:
        self._assignment: FleetAssignment | None = None

    @property
    def assignment(self) -> FleetAssignment | None:
        return self._assignment

    @property
    def frozen(self) -> bool:
        return self._assignment is not None

    def freeze(self, assignment: FleetAssignment) -> FleetAssignment:
        if self._assignment is None:
            self._assignment = assignment
            return assignment
        if dict(self._assignment.vehicle_to_plate) != dict(assignment.vehicle_to_plate):
            raise RuntimeError("fleet assignment is already frozen for this mission epoch")
        return self._assignment


@dataclass(frozen=True)
class TwoDroneAssignment:
    """Frozen binding for one diametric pair and one vehicle ordering."""

    vehicle_to_plate: Mapping[str, int]
    opposite_pair_index: int
    total_cost: float = math.inf
    candidate_count: int = 12

    def __post_init__(self) -> None:
        mapping = {str(key): int(value) for key, value in dict(self.vehicle_to_plate).items()}
        if len(mapping) != 2 or len(set(mapping.values())) != 2:
            raise ValueError("two-drone assignment requires two unique bindings")
        values = tuple(mapping.values())
        if (values[0] - values[1]) % 12 != 6:
            raise ValueError("two-drone plates must be diametrically opposite")
        pair_index = int(self.opposite_pair_index)
        if pair_index not in range(len(OPPOSITE_PLATE_PAIRS)):
            raise ValueError("opposite_pair_index out of range")
        if set(values) != set(OPPOSITE_PLATE_PAIRS[pair_index]):
            raise ValueError("binding does not match opposite_pair_index")
        cost = float(self.total_cost)
        if not (math.isfinite(cost) or math.isinf(cost)) or cost < 0.0:
            raise ValueError("total_cost must be non-negative")
        object.__setattr__(self, "vehicle_to_plate", MappingProxyType(mapping))
        object.__setattr__(self, "opposite_pair_index", pair_index)
        object.__setattr__(self, "total_cost", cost)
        object.__setattr__(self, "candidate_count", int(self.candidate_count))


class TwoDroneAssignmentLatch:
    def __init__(self) -> None:
        self._assignment: TwoDroneAssignment | None = None

    @property
    def assignment(self) -> TwoDroneAssignment | None:
        return self._assignment

    @property
    def frozen(self) -> bool:
        return self._assignment is not None

    def freeze(self, assignment: TwoDroneAssignment) -> TwoDroneAssignment:
        if self._assignment is None:
            self._assignment = assignment
        elif dict(self._assignment.vehicle_to_plate) != dict(assignment.vehicle_to_plate):
            raise RuntimeError("two-drone assignment is already frozen")
        return self._assignment


def enumerate_two_drone_opposite_assignments(
    vehicle_ids: Iterable[str],
) -> tuple[TwoDroneAssignment, ...]:
    ordered = tuple(sorted(str(value) for value in vehicle_ids))
    if len(ordered) != 2 or len(set(ordered)) != 2:
        raise ValueError("exactly two unique vehicle IDs are required")
    result: list[TwoDroneAssignment] = []
    for pair_index, pair in enumerate(OPPOSITE_PLATE_PAIRS):
        for permutation in (pair, tuple(reversed(pair))):
            result.append(
                TwoDroneAssignment(
                    vehicle_to_plate=dict(zip(ordered, permutation)),
                    opposite_pair_index=pair_index,
                )
            )
    return tuple(result)


def enumerate_candidate_assignments(vehicle_ids: Iterable[str]) -> tuple[FleetAssignment, ...]:
    ordered_ids = tuple(sorted(str(value) for value in vehicle_ids))
    if len(ordered_ids) != 4 or len(set(ordered_ids)) != 4:
        raise ValueError("exactly four unique vehicle IDs are required")
    result: list[FleetAssignment] = []
    for set_index, plate_set in enumerate(SYMMETRIC_PLATE_SETS):
        for permutation in itertools.permutations(plate_set):
            result.append(
                FleetAssignment(
                    vehicle_to_plate=dict(zip(ordered_ids, permutation)),
                    plate_set_index=set_index,
                )
            )
    return tuple(result)


def attached_hold_body_target_world(
    *,
    geometry: RingNetGeometry,
    ring_position_world,
    rotation_world_from_ring,
    plate_index: int,
    vehicle_yaw_rad: float,
    tether_length_m: float,
    tether_anchor_body,
    attached_hold_angle_deg: float,
) -> np.ndarray:
    """Reuse the exact B1 contact-to-body equation for an arbitrary assigned plate."""

    ring_position = _vec3(ring_position_world, name="ring_position_world")
    ring_rotation = _rotation(rotation_world_from_ring, name="rotation_world_from_ring")
    cfg = M2BConfig(attached_hold_angle_deg=float(attached_hold_angle_deg))
    plate_position = geometry.plate_position_world(
        ring_position=ring_position,
        rotation_world_from_ring=ring_rotation,
        plate_index=int(plate_index),
    )
    plate_rotation = geometry.plate_rotation_world(
        rotation_world_from_ring=ring_rotation,
        plate_index=int(plate_index),
    )
    direction_world = plate_rotation @ attached_hold_direction_plate(cfg)
    return body_reference_from_contact(
        contact_position_world=plate_position,
        direction_contact_to_vehicle_world=direction_world,
        tether_length_m=float(tether_length_m),
        rotation_world_from_body=_yaw_rotation(vehicle_yaw_rad),
        tether_anchor_body=_vec3(tether_anchor_body, name="tether_anchor_body"),
    )


def validate_fleet_states(
    vehicles: Sequence[FleetVehicleState],
    *,
    now_s: float,
    config: FleetAssignmentConfig,
) -> tuple[FleetVehicleState, ...]:
    now = float(now_s)
    if not math.isfinite(now):
        raise ValueError("now_s must be finite")
    if len(vehicles) != 4:
        raise RuntimeError("exactly four vehicle states are required before assignment")
    ordered = tuple(sorted(vehicles, key=lambda item: item.vehicle_id))
    identifiers = [item.vehicle_id for item in ordered]
    physical = [item.physical_drone_id for item in ordered]
    namespaces = [item.namespace for item in ordered]
    rigid_bodies = [item.mocap_rigid_body_id for item in ordered]
    for label, values in (
        ("vehicle_id", identifiers),
        ("physical_drone_id", physical),
        ("namespace", namespaces),
        ("mocap_rigid_body_id", rigid_bodies),
    ):
        if len(set(values)) != 4:
            raise RuntimeError(f"duplicate fleet {label}")
    for state in ordered:
        age = now - state.stamp_s
        if age < -1e-6:
            raise RuntimeError(f"future-dated state for {state.vehicle_id}")
        if age > config.state_timeout_s:
            raise RuntimeError(f"stale vehicle state for {state.vehicle_id}: age={age:.3f}s")
    return ordered


def choose_fleet_assignment(
    *,
    vehicles: Sequence[FleetVehicleState],
    ring_position_world,
    rotation_world_from_ring,
    now_s: float,
    geometry: RingNetGeometry | None = None,
    config: FleetAssignmentConfig | None = None,
    ring_stamp_s: float | None = None,
) -> FleetAssignment:
    cfg = FleetAssignmentConfig() if config is None else config
    ring_position = _vec3(ring_position_world, name="ring_position_world")
    ring_rotation = _rotation(rotation_world_from_ring, name="rotation_world_from_ring")
    now = float(now_s)
    ordered = validate_fleet_states(vehicles, now_s=now, config=cfg)
    if ring_stamp_s is not None:
        ring_age = now - float(ring_stamp_s)
        if not math.isfinite(ring_age) or ring_age < -1e-6 or ring_age > cfg.ring_state_timeout_s:
            raise RuntimeError(f"ring state is stale/invalid: age={ring_age:.3f}s")
    ring_geometry = RingNetGeometry() if geometry is None else geometry
    states = {item.vehicle_id: item for item in ordered}
    candidates = enumerate_candidate_assignments(states)
    scored: list[tuple[float, int, tuple[int, ...], FleetAssignment]] = []
    ordered_ids = tuple(sorted(states))
    for candidate in candidates:
        mapping = dict(candidate.vehicle_to_plate)
        total = 0.0
        for vehicle_id in ordered_ids:
            state = states[vehicle_id]
            target = attached_hold_body_target_world(
                geometry=ring_geometry,
                ring_position_world=ring_position,
                rotation_world_from_ring=ring_rotation,
                plate_index=mapping[vehicle_id],
                vehicle_yaw_rad=state.yaw_rad,
                tether_length_m=cfg.tether_length_m,
                tether_anchor_body=cfg.tether_anchor_body,
                attached_hold_angle_deg=cfg.attached_hold_angle_deg,
            )
            error = state.position_world - target
            total += float(error @ error)
        lexical_mapping = tuple(mapping[vehicle_id] for vehicle_id in ordered_ids)
        scored_candidate = FleetAssignment(
            vehicle_to_plate=mapping,
            plate_set_index=candidate.plate_set_index,
            total_cost=total,
            candidate_count=len(candidates),
        )
        scored.append((total, candidate.plate_set_index, lexical_mapping, scored_candidate))
    # Round only for tie classification, never for reported cost. This keeps replay
    # deterministic when symmetric floating-point geometry differs at ~machine epsilon.
    best = min(scored, key=lambda row: (round(row[0], 12), row[1], row[2]))
    return best[3]


def choose_two_drone_assignment(
    *,
    vehicles: Sequence[FleetVehicleState],
    ring_position_world,
    rotation_world_from_ring,
    now_s: float,
    geometry: RingNetGeometry | None = None,
    config: FleetAssignmentConfig | None = None,
    ring_stamp_s: float | None = None,
) -> TwoDroneAssignment:
    """Score all six opposite pairs and both vehicle permutations (12 total)."""

    cfg = FleetAssignmentConfig() if config is None else config
    if len(vehicles) != 2:
        raise RuntimeError("exactly two vehicle states are required before assignment")
    ordered = tuple(sorted(vehicles, key=lambda item: item.vehicle_id))
    if len({item.vehicle_id for item in ordered}) != 2:
        raise RuntimeError("duplicate vehicle_id")
    if len({item.physical_drone_id for item in ordered}) != 2:
        raise RuntimeError("duplicate physical_drone_id")
    if len({item.mocap_rigid_body_id for item in ordered}) != 2:
        raise RuntimeError("duplicate mocap_rigid_body_id")
    now = float(now_s)
    for state in ordered:
        age = now - state.stamp_s
        if age < -1e-6 or age > cfg.state_timeout_s:
            raise RuntimeError(f"stale/invalid state for {state.vehicle_id}: age={age:.3f}s")
    if ring_stamp_s is not None:
        ring_age = now - float(ring_stamp_s)
        if ring_age < -1e-6 or ring_age > cfg.ring_state_timeout_s:
            raise RuntimeError(f"ring state is stale/invalid: age={ring_age:.3f}s")

    ring_position = _vec3(ring_position_world, name="ring_position_world")
    ring_rotation = _rotation(rotation_world_from_ring, name="rotation_world_from_ring")
    ring_geometry = RingNetGeometry() if geometry is None else geometry
    states = {item.vehicle_id: item for item in ordered}
    ordered_ids = tuple(sorted(states))
    candidates = enumerate_two_drone_opposite_assignments(ordered_ids)
    scored: list[tuple[float, int, tuple[int, ...], TwoDroneAssignment]] = []
    for candidate in candidates:
        mapping = dict(candidate.vehicle_to_plate)
        total = 0.0
        for vehicle_id in ordered_ids:
            state = states[vehicle_id]
            target = attached_hold_body_target_world(
                geometry=ring_geometry,
                ring_position_world=ring_position,
                rotation_world_from_ring=ring_rotation,
                plate_index=mapping[vehicle_id],
                vehicle_yaw_rad=state.yaw_rad,
                tether_length_m=cfg.tether_length_m,
                tether_anchor_body=cfg.tether_anchor_body,
                attached_hold_angle_deg=cfg.attached_hold_angle_deg,
            )
            error = state.position_world - target
            total += float(error @ error)
        lexical = tuple(mapping[vehicle_id] for vehicle_id in ordered_ids)
        scored_candidate = TwoDroneAssignment(
            vehicle_to_plate=mapping,
            opposite_pair_index=candidate.opposite_pair_index,
            total_cost=total,
            candidate_count=len(candidates),
        )
        scored.append((total, candidate.opposite_pair_index, lexical, scored_candidate))
    return min(scored, key=lambda row: (round(row[0], 12), row[1], row[2]))[3]
