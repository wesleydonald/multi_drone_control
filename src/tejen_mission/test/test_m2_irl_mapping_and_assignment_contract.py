"""Focused contract for the minimum two-vehicle M2 IRL hardware boundary."""

from __future__ import annotations

import ast
import importlib
from pathlib import Path

import numpy as np

from tejen_mission.cooperative_trajectory import RingNetGeometry


def _hardware():
    return importlib.import_module("tejen_mission.m2_irl_hardware")


def _assignment():
    return importlib.import_module("tejen_mission.m2_fleet_assignment")


def test_exact_two_drone_hardware_mapping_and_shared_udp_endpoint():
    hw = _hardware()
    mapping = hw.default_two_drone_hardware()

    assert tuple(mapping) == ("drone_0", "drone_1")
    assert mapping["drone_0"].physical_quad == 2
    assert mapping["drone_0"].body_id == 12
    assert mapping["drone_0"].magnet_id == 22
    assert mapping["drone_0"].elrs_device == "/dev/QUAD2"
    assert mapping["drone_0"].ftdi_serial == "FTF0AROT"
    assert mapping["drone_1"].physical_quad == 4
    assert mapping["drone_1"].body_id == 14
    assert mapping["drone_1"].magnet_id == 24
    assert mapping["drone_1"].elrs_device == "/dev/QUAD4"
    assert mapping["drone_1"].ftdi_serial == "FTF09R63"
    assert hw.RING_RIGID_BODY_ID == 8
    assert hw.DEFAULT_MOCAP_BIND_ADDRESS == "0.0.0.0"
    assert hw.DEFAULT_MOCAP_BIND_PORT == 1511
    assert not {11, 13}.intersection(hw.required_rigid_body_ids(mapping))


def test_per_body_throttle_is_independent_and_caps_before_parse():
    hw = _hardware()
    throttle = hw.PerRigidBodyThrottle(max_rate_hz=120.0)

    assert throttle.accept(12, 0.000)
    assert throttle.accept(14, 0.000), "body 12 must not throttle body 14"
    assert not throttle.accept(12, 0.004)
    assert throttle.accept(12, 0.009)

    router_path = (
        Path(__file__).resolve().parents[1]
        / "tejen_mission"
        / "m2_irl_mocap_router.py"
    )
    tree = ast.parse(router_path.read_text(encoding="utf-8"))
    source = router_path.read_text(encoding="utf-8")
    assert source.count("socket.socket(") == 1
    poll_source = ast.get_source_segment(
        source,
        next(
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.FunctionDef) and node.name == "_poll_socket"
        ),
    )
    assert poll_source.index("throttle.accept") < poll_source.index("parser.parse")
    assert sum(
        isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == "main"
        for node in ast.walk(tree)
    ) == 1


def test_exactly_twelve_two_drone_opposite_plate_candidates():
    module = _assignment()
    candidates = module.enumerate_two_drone_opposite_assignments(
        ("drone_0", "drone_1")
    )

    assert len(candidates) == 12
    observed = set()
    for candidate in candidates:
        mapping = dict(candidate.vehicle_to_plate)
        assert set(mapping) == {"drone_0", "drone_1"}
        a, b = mapping["drone_0"], mapping["drone_1"]
        assert (a - b) % 12 == 6
        observed.add((a, b))
    assert len(observed) == 12


def test_two_drone_assignment_uses_existing_hold_target_cost_and_freezes():
    module = _assignment()
    geometry = RingNetGeometry()
    ring = np.array([0.2, -0.1, 0.035])
    config = module.FleetAssignmentConfig()
    desired = {"drone_0": 2, "drone_1": 8}
    vehicles = []
    for index, vehicle_id in enumerate(("drone_0", "drone_1")):
        target = module.attached_hold_body_target_world(
            geometry=geometry,
            ring_position_world=ring,
            rotation_world_from_ring=np.eye(3),
            plate_index=desired[vehicle_id],
            vehicle_yaw_rad=0.0,
            tether_length_m=config.tether_length_m,
            tether_anchor_body=config.tether_anchor_body,
            attached_hold_angle_deg=config.attached_hold_angle_deg,
        )
        vehicles.append(
            module.FleetVehicleState(
                vehicle_id=vehicle_id,
                physical_drone_id=(2, 4)[index],
                namespace=f"/{vehicle_id}",
                mocap_rigid_body_id=(12, 14)[index],
                position_world=target,
                yaw_rad=0.0,
                stamp_s=5.0,
            )
        )

    chosen = module.choose_two_drone_assignment(
        vehicles=vehicles,
        ring_position_world=ring,
        rotation_world_from_ring=np.eye(3),
        now_s=5.1,
        geometry=geometry,
        config=config,
        ring_stamp_s=5.0,
    )
    assert dict(chosen.vehicle_to_plate) == desired
    assert chosen.candidate_count == 12
    latch = module.TwoDroneAssignmentLatch()
    assert latch.freeze(chosen) is chosen
    other = next(
        item
        for item in module.enumerate_two_drone_opposite_assignments(
            ("drone_0", "drone_1")
        )
        if dict(item.vehicle_to_plate) != desired
    )
    try:
        latch.freeze(other)
    except RuntimeError:
        pass
    else:
        raise AssertionError("the pre-flight assignment must be immutable")
