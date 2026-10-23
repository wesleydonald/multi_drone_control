"""Four-drone wiring reuses the proven two-drone mission behaviour."""
from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest
import yaml

from tejen_mission import m2_irl_launch_config as config
from tejen_mission.m2_irl_readiness import TwoDroneIRLReadinessGate

PKG = Path(__file__).resolve().parents[1]
IDS = tuple(f"drone_{i}" for i in range(4))
SERIALS = ("FTF0AR84", "FTF0AROT", "FTF09W4A", "FTF09R63")


def four_payload():
    payload = yaml.safe_load((PKG / "config/m2_irl_two_drone.yaml").read_text())
    payload["m2_irl"]["vehicles"] = {
        v: dict(physical_quad=i+1, body_id=11+i, magnet_id=21+i,
                ftdi_serial=SERIALS[i], serial_port=f"/dev/QUAD{i+1}",
                magnet_channel=6, props_off_verified=True)
        for i, v in enumerate(IDS)
    }
    return payload


def test_four_identity_roundtrip_and_rejects_duplicate_or_partial_fleet():
    payload = four_payload()
    identity = config.parse_fleet_identity(payload)
    assert tuple(identity.vehicles) == IDS
    assert [v.physical_quad for v in identity.vehicles.values()] == [1, 2, 3, 4]
    assert [v.body_id for v in identity.vehicles.values()] == [11, 12, 13, 14]
    assert [v.magnet_id for v in identity.vehicles.values()] == [21, 22, 23, 24]
    encoded = config.identity_ros_parameters(identity)["hardware_identity_json"]
    assert config.parse_identity_json(encoded) == identity
    with pytest.raises(ValueError):
        config.parse_two_drone_identity(payload)
    for mutate in (
        lambda v: v.pop("drone_3"),
        lambda v: v["drone_3"].update(body_id=11),
        lambda v: v["drone_3"].update(ftdi_serial=SERIALS[0]),
        lambda v: v["drone_3"].update(serial_port="/dev/QUAD1"),
    ):
        bad = deepcopy(payload)
        mutate(bad["m2_irl"]["vehicles"])
        with pytest.raises(ValueError):
            config.parse_fleet_identity(bad)


def test_four_config_preserves_proven_profiles_and_requires_new_magnet_checks():
    payload = yaml.safe_load((PKG / "config/m2_irl_four_drone.yaml").read_text())
    original = yaml.safe_load((PKG / "config/m2_irl_two_drone.yaml").read_text())
    for key in ("geometry", "gate_profiles", "attachment_detector", "mocap", "attachment_policy"):
        assert payload["m2_irl"][key] == original["m2_irl"][key]
    identity = config.parse_fleet_identity(payload)
    assert [v.ftdi_serial for v in identity.vehicles.values()] == list(SERIALS)
    # Operator commissioning flags may legitimately become true in the lab.
    payload["m2_irl"]["vehicles"]["drone_2"]["props_off_verified"] = False
    with pytest.raises(ValueError, match="props-off"):
        config.validate_mission_hardware_config(payload)
    for vehicle in payload["m2_irl"]["vehicles"].values():
        vehicle["props_off_verified"] = True
    assert config.validate_mission_hardware_config(payload) == {
        v: (f"/dev/QUAD{i+1}", 6) for i, v in enumerate(IDS)
    }


def test_four_readiness_needs_every_vehicle_and_inbound_link():
    gate = TwoDroneIRLReadinessGate(vehicle_ids=IDS, stationary_dwell_s=0.)
    gate.update_ring(stamp_s=10.)
    for v in IDS:
        gate.update_vehicle(v, body_stamp_s=10., magnet_stamp_s=10.,
            speed_mps=0., armed=False, arming_stamp_s=10., planner_stamp_s=10.,
            backend_stamp_s=10., controller_stamp_s=10., telemetry_stamp_s=10.)
        assert not gate.evaluate(10.).ready
        gate.note_link_statistics(v, stamp_s=10.)
    assert gate.evaluate(10.).ready
    assert set(gate.evaluate(10.).vehicle_ready) == set(IDS)
    assert not gate.evaluate(11.).ready


@pytest.mark.parametrize("flight_order", [None, ("drone_2", "drone_0", "drone_1", "drone_3")])
@pytest.mark.parametrize("completion_mode", ["airborne_hold", "attach_park"])
def test_four_launch_builds_complete_stacks_and_three_peers(tmp_path, flight_order, completion_mode):
    from launch import LaunchContext
    from launch_ros.utilities import evaluate_parameters
    from launch_ros.actions import Node
    from launch.actions import TimerAction
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location("m2_four_launch", PKG / "launch/m2_irl_two_drone.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    path = tmp_path / "four.yaml"
    payload = four_payload()
    if flight_order is not None:
        payload["m2_irl"]["flight_order"] = list(flight_order)
    path.write_text(yaml.safe_dump(payload))
    context = LaunchContext()
    context.launch_configurations.update(mode="mission", config_file=str(path),
        log_dir=str(tmp_path / "logs"), rviz="true", expected_vehicle_count="4", completion_mode=completion_mode)
    actions = module._launch_setup(context)
    nodes = [a for a in actions if isinstance(a, Node)]
    timers = [a for a in actions if isinstance(a, TimerAction)]
    nodes += [n for a in timers for n in a.actions if isinstance(n, Node)]
    by_executable = {}
    for n in nodes:
        exe = n.node_executable if isinstance(n.node_executable, str) else perform_substitutions(context, n.node_executable)
        params = {}
        for p in evaluate_parameters(context, n._Node__parameters):
            if isinstance(p, dict):
                params.update(p)
        by_executable.setdefault(exe, []).append(params)
    for exe in ("elrs_interface_irl", "m2_attachment_observer", "online_join_planner",
                "dynamic_planner_transfer_backend", "main"):
        assert len(by_executable[exe]) == 4, exe
    assert len(by_executable["m2_irl_mocap_router"]) == 1
    supervisor = by_executable["m2d_fleet_supervisor"][0]
    assert list(supervisor["vehicle_ids"]) == list(flight_order or IDS)
    assert supervisor["target_attachment_count"] == 4
    assert supervisor["completion_mode"] == completion_mode
    for planner in by_executable["online_join_planner"]:
        assert planner["m2b_completion_mode"] == completion_mode
        if completion_mode == "attach_park":
            assert planner["parking_max_radius_m"] == .50
            assert planner["parking_disarm_height_above_rest_m"] == .02
    for p in by_executable["dynamic_planner_transfer_backend"]:
        peers = [v for v in IDS if v != p["vehicle_id"]]
        assert list(p["cooperative_vehicle_ids"]) == peers
        for key, suffix in (
            ("cooperative_drone_state_topics", "motion_capture_state"),
            ("cooperative_committed_trajectory_topics", "committed_trajectory"),
            ("cooperative_attached_plate_topics", "m2d/attached_plate_id"),
        ):
            assert list(p[key]) == [f"/{v}/{suffix}" for v in peers]
    for i, p in enumerate(by_executable["elrs_interface_irl"]):
        assert p["serial_port"] == f"/dev/QUAD{i+1}"
    assert "rviz2" in by_executable
    for p in by_executable["main"]:
        assert p["require_external_arm_permission"] is True
    assert by_executable["m2_irl_mocap_router"][0]["hardware_identity_json"] == by_executable["m2_irl_fleet_manager"][0]["hardware_identity_json"]
    if completion_mode == "attach_park":
        payload["m2_irl"]["completion_mode"] = completion_mode
    assert yaml.safe_load((tmp_path / "logs/resolved_config.yaml").read_text()) == payload
    context.launch_configurations["expected_vehicle_count"] = "2"
    with pytest.raises(ValueError, match="vehicle count"):
        module._launch_setup(context)


def test_configured_flight_order_grants_quad3_first_and_reverses_landing():
    from tejen_mission.m2d_sequential_supervision import M2DSequentialSupervisorCore

    payload = yaml.safe_load((PKG / "config/m2_irl_four_drone.yaml").read_text())
    order = config.parse_flight_order(payload)
    assert order == ("drone_2", "drone_0", "drone_1", "drone_3")
    core = M2DSequentialSupervisorCore(vehicle_ids=order, operator_hold_after_goal=True,
                                     progressive_policy=True)
    core.freeze_assignment(dict(zip(IDS, (0, 3, 6, 9))))
    core.update_m2c_ready(True)
    for i, vehicle in enumerate(order):
        snap = core.snapshot()
        assert snap.active_vehicle_id == vehicle
        assert {v for v, permitted in snap.permissions.items() if permitted} == {vehicle}
        core.update_phase(vehicle, "M2_ATTACHED_HOLD")
        for attached in order[:i+1]:
            core.update_attachment_evidence(attached, confirmed=True, lost=False, stamp_s=float(i))
        core.step(float(i))
    assert core.snapshot().success_hold
    assert core.request_landing()
    assert core.landing_order == tuple(reversed(order))
    original = yaml.safe_load((PKG / "config/m2_irl_two_drone.yaml").read_text())
    assert config.parse_flight_order(original) == ("drone_0", "drone_1")
    for invalid in (["drone_2"]*4, ["drone_0"], ["drone_0","drone_1","drone_2","drone_4"]):
        payload["m2_irl"]["flight_order"] = invalid
        with pytest.raises(ValueError, match="flight_order"):
            config.parse_flight_order(payload)
