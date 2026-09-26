"""Focused two-drone IRL readiness and supervisor configuration contract."""

from __future__ import annotations

import importlib
from pathlib import Path
import re


def _readiness():
    return importlib.import_module("tejen_mission.m2_irl_readiness")


def _feed(gate, now: float, *, armed_1: bool = False, speed_1: float = 0.0, link_stats: bool = True):
    gate.update_ring(stamp_s=now)
    for vehicle_id in ("drone_0", "drone_1"):
        gate.update_vehicle(
            vehicle_id,
            body_stamp_s=now,
            magnet_stamp_s=now,
            speed_mps=speed_1 if vehicle_id == "drone_1" else 0.0,
            armed=armed_1 if vehicle_id == "drone_1" else False,
            arming_stamp_s=now,
            planner_stamp_s=now,
            backend_stamp_s=now,
            controller_stamp_s=now,
            telemetry_stamp_s=now,
        )
        if link_stats:
            gate.note_link_statistics(vehicle_id, stamp_s=now)


def test_readiness_requires_exactly_two_named_vehicles_and_full_dwell():
    api = _readiness()
    gate = api.TwoDroneIRLReadinessGate(
        vehicle_ids=("drone_0", "drone_1"), timeout_s=0.25, stationary_dwell_s=1.0
    )
    _feed(gate, 10.0)
    assert not gate.evaluate(10.0).ready
    _feed(gate, 10.99)
    assert not gate.evaluate(10.99).ready
    _feed(gate, 11.0)
    snapshot = gate.evaluate(11.0)
    assert snapshot.ready
    assert set(snapshot.vehicle_ready) == {"drone_0", "drone_1"}


def test_readiness_fails_closed_for_armed_motion_stale_or_missing_evidence():
    api = _readiness()
    gate = api.TwoDroneIRLReadinessGate(stationary_dwell_s=0.0)
    _feed(gate, 2.0, armed_1=True)
    assert not gate.evaluate(2.0).ready
    _feed(gate, 2.1, speed_1=0.2)
    assert not gate.evaluate(2.1).ready
    _feed(gate, 3.0)
    assert gate.evaluate(3.0).ready
    assert not gate.evaluate(3.30).ready

    try:
        gate.update_vehicle(
            "drone_2",
            body_stamp_s=3.0,
            magnet_stamp_s=3.0,
            speed_mps=0.0,
            armed=False,
            arming_stamp_s=3.0,
            planner_stamp_s=3.0,
            backend_stamp_s=3.0,
            controller_stamp_s=3.0,
            telemetry_stamp_s=3.0,
        )
    except KeyError:
        pass
    else:
        raise AssertionError("two-drone IRL readiness must reject out-of-scope drones")




def test_readiness_reports_the_specific_failed_vehicle_predicate():
    gate = _readiness().TwoDroneIRLReadinessGate(stationary_dwell_s=0.0)
    _feed(gate, 2.0, armed_1=True, speed_1=0.2, link_stats=False)
    snapshot = gate.evaluate(2.0)
    assert snapshot.vehicle_reasons["drone_0"] == ("link_statistics_stale",)
    assert snapshot.vehicle_reasons["drone_1"] == (
        "link_statistics_stale", "armed", "moving",
    )


def test_periodic_telemetry_alone_cannot_make_fleet_ready():
    gate = _readiness().TwoDroneIRLReadinessGate(stationary_dwell_s=0.0)
    _feed(gate, 10.0, link_stats=False)
    assert not gate.evaluate(10.0).ready
    gate.note_link_statistics("drone_0", stamp_s=10.0)
    assert not gate.evaluate(10.0).ready
    gate.note_link_statistics("drone_1", stamp_s=10.0)
    assert gate.evaluate(10.0).ready
    _feed(gate, 10.51, link_stats=False)
    assert not gate.evaluate(10.51).ready


def test_slow_one_hz_health_heartbeats_use_a_separate_bounded_timeout():
    api = _readiness()
    gate = api.TwoDroneIRLReadinessGate(
        timeout_s=0.25, status_timeout_s=1.25, stationary_dwell_s=0.0
    )
    gate.update_ring(stamp_s=11.0)
    for vehicle_id in ("drone_0", "drone_1"):
        gate.update_vehicle(
            vehicle_id,
            body_stamp_s=11.0,
            magnet_stamp_s=11.0,
            speed_mps=0.0,
            armed=False,
            arming_stamp_s=11.0,
            planner_stamp_s=10.0,
            backend_stamp_s=10.0,
            controller_stamp_s=10.0,
            telemetry_stamp_s=11.0,
        )
        gate.note_link_statistics(vehicle_id, stamp_s=11.0)
    assert gate.evaluate(11.0).ready
    gate.update_ring(stamp_s=11.30)
    assert not gate.evaluate(11.30).ready

def test_fresh_link_statistics_helper_fails_closed_at_arm_time():
    api = _readiness()
    assert not api.link_statistics_fresh(None, now_s=10.0, timeout_s=0.5)
    assert api.link_statistics_fresh(9.6, now_s=10.0, timeout_s=0.5)
    assert not api.link_statistics_fresh(9.4, now_s=10.0, timeout_s=0.5)
    assert not api.link_statistics_fresh(10.1, now_s=10.0, timeout_s=0.5)


def test_irl_planner_arm_guard_checks_live_link_but_sim_default_is_off():
    planner = (
        Path(__file__).resolve().parents[1]
        / "tejen_mission"
        / "online_join_planner.py"
    ).read_text(encoding="utf-8")
    launch = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "m2_irl_two_drone.launch.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("m2b_require_recent_link_statistics", False)' in planner
    assert "self.m2b_link_statistics_callback" in planner
    arm = planner.split("def m2b_arm_permitted", 1)[1].split(
        "def update_m2b_bootstrap", 1
    )[0]
    assert "link_statistics_fresh(" in arm
    assert '"m2b_require_recent_link_statistics": True' in launch


def test_planner_arm_permission_revokes_when_inbound_link_event_stales():
    module = importlib.import_module("tejen_mission.online_join_planner")
    planner = object.__new__(module.OnlineJoinPlanner)
    planner.is_m2b = True
    planner.m2d_enabled = False
    planner.phase = module.MissionPhase.M2_WAIT_FOR_ARM
    planner.m2b_bootstrap_arm_permitted = True
    planner.m2b_require_recent_link_statistics = True
    planner.m2b_link_statistics_timeout_s = 0.5
    planner.m2b_last_link_statistics_s = None
    planner._physical_now_s = lambda: 10.0
    assert not planner.m2b_arm_permitted()
    planner.m2b_last_link_statistics_s = 9.8
    assert planner.m2b_arm_permitted()
    planner.m2b_last_link_statistics_s = 9.4
    assert not planner.m2b_arm_permitted()
    planner.m2b_require_recent_link_statistics = False
    assert planner.m2b_arm_permitted()


def test_supervisor_wrapper_declares_vehicle_ids_and_keeps_four_drone_default():
    path = (
        Path(__file__).resolve().parents[1]
        / "tejen_mission"
        / "m2d_fleet_supervisor.py"
    )
    source = path.read_text(encoding="utf-8")
    assert re.search(
        r'declare_parameter\(\s*"vehicle_ids",\s*list\(DEFAULT_VEHICLE_IDS\)\s*\)',
        source,
    )
    assert "self.vehicle_ids = tuple(" in source
    assert "vehicle_ids=self.vehicle_ids" in source
    assert "order 0->1->2->3" not in source


def test_irl_manager_uses_fresh_preassignment_planner_output_without_commitment_cycle():
    path = (
        Path(__file__).resolve().parents[1]
        / "tejen_mission"
        / "m2_irl_fleet_manager.py"
    )
    source = path.read_text(encoding="utf-8")

    assert 'Bool, f"{ns}/join_planner/arm_permission"' in source
    assert 'CommittedTrajectory, f"{ns}/committed_trajectory"' not in source
