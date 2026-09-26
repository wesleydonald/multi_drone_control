from __future__ import annotations

import pytest

from tejen_mission.m2d_sequential_supervision import M2DSequentialSupervisorCore


ASSIGNMENT = {"drone_0": 7, "drone_1": 1, "drone_2": 10, "drone_3": 4}


def _ready_core():
    core = M2DSequentialSupervisorCore(evidence_timeout_s=0.5)
    core.freeze_assignment(ASSIGNMENT)
    core.update_m2c_ready(True)
    return core


def _attach(core, vehicle_id: str, now_s: float):
    # In the real node every observer continues publishing while later vehicles
    # execute, so keep already-proven evidence fresh in this synthetic sequence.
    for attached_vehicle in tuple(core.attached):
        core.update_attachment_evidence(
            attached_vehicle, confirmed=True, lost=False, stamp_s=now_s
        )
    core.update_phase(vehicle_id, "M2_ATTACHED_HOLD")
    core.update_attachment_evidence(
        vehicle_id, confirmed=True, lost=False, stamp_s=now_s
    )
    return core.step(now_s)


def test_m2d_does_not_grant_permission_before_frozen_assignment_and_m2c_ready():
    core = M2DSequentialSupervisorCore()
    assert not any(core.snapshot().permissions.values())
    core.freeze_assignment(ASSIGNMENT)
    assert core.snapshot().state == "WAIT_M2C_READY"
    assert not any(core.snapshot().permissions.values())
    core.update_m2c_ready(True)
    snapshot = core.step(1.0)
    assert snapshot.state == "ACTIVE_0"
    assert snapshot.permissions == {
        "drone_0": True,
        "drone_1": False,
        "drone_2": False,
        "drone_3": False,
    }


def test_m2d_assignment_is_frozen_and_cannot_be_reassigned_in_flight():
    core = _ready_core()
    other = dict(ASSIGNMENT)
    other["drone_0"], other["drone_1"] = other["drone_1"], other["drone_0"]
    with pytest.raises(RuntimeError):
        core.freeze_assignment(other)


def test_m2d_advances_exactly_0_1_2_3_on_software_proven_attached_hold():
    core = _ready_core()
    for index, vehicle_id in enumerate(("drone_0", "drone_1", "drone_2", "drone_3")):
        before = core.snapshot()
        assert before.active_vehicle_id == vehicle_id
        snap = _attach(core, vehicle_id, float(index + 1))
        if index < 3:
            assert snap.active_vehicle_id == f"drone_{index + 1}"
            assert snap.permissions[f"drone_{index + 1}"] is True
        else:
            assert snap.passed is True
            assert not any(snap.permissions.values())
    assert core.snapshot().attached_plate_ids == ASSIGNMENT


def test_m2d_phase_without_fresh_software_confirmation_does_not_advance():
    core = _ready_core()
    core.update_phase("drone_0", "M2_ATTACHED_HOLD")
    assert core.step(1.0).active_vehicle_id == "drone_0"
    core.update_attachment_evidence("drone_0", confirmed=True, lost=False, stamp_s=0.0)
    assert core.step(1.0).active_vehicle_id == "drone_0"


def test_m2d_gazebo_joint_truth_is_not_part_of_progression_api():
    core = _ready_core()
    with pytest.raises(TypeError):
        core.update_attachment_evidence(  # type: ignore[call-arg]
            "drone_0", confirmed=True, lost=False, stamp_s=1.0, joint_detached=False
        )


def test_m2d_active_terminal_failure_aborts_without_reassignment_or_next_permission():
    core = _ready_core()
    core.update_phase("drone_0", "M2_FAULT")
    snap = core.step(1.0)
    assert snap.aborted is True
    assert "drone_0" in snap.abort_reason
    assert not any(snap.permissions.values())
    assert snap.assigned_plate_ids == ASSIGNMENT


def test_m2d_loss_of_previously_proven_attachment_is_latched_fleet_abort():
    core = _ready_core()
    _attach(core, "drone_0", 1.0)
    assert core.snapshot().active_vehicle_id == "drone_1"
    core.update_attachment_evidence("drone_0", confirmed=False, lost=True, stamp_s=1.1)
    snap = core.step(1.1)
    assert snap.aborted is True
    assert snap.abort_reason == "PROVEN_ATTACHMENT_LOST:drone_0"
    assert not any(snap.permissions.values())


def test_m2d_m2c_ready_is_latched_and_may_drop_after_first_takeoff():
    core = _ready_core()
    core.update_m2c_ready(False)
    assert core.step(1.0).active_vehicle_id == "drone_0"


def test_m2d_operator_hold_can_stop_after_two_attachments_without_granting_drone_2():
    core = M2DSequentialSupervisorCore(
        evidence_timeout_s=0.5,
        target_attachment_count=2,
        operator_hold_after_goal=True,
    )
    core.freeze_assignment(ASSIGNMENT)
    core.update_m2c_ready(True)

    _attach(core, "drone_0", 1.0)
    snap = _attach(core, "drone_1", 2.0)

    assert snap.success_hold is True
    assert snap.active_vehicle_id is None
    assert not any(snap.permissions.values())
    assert snap.attached_plate_ids == {
        "drone_0": ASSIGNMENT["drone_0"],
        "drone_1": ASSIGNMENT["drone_1"],
    }


def test_m2d_operator_landing_is_reverse_sequential_and_allows_intentional_detach():
    core = M2DSequentialSupervisorCore(
        evidence_timeout_s=0.5,
        target_attachment_count=2,
        operator_hold_after_goal=True,
    )
    core.freeze_assignment(ASSIGNMENT)
    core.update_m2c_ready(True)
    _attach(core, "drone_0", 1.0)
    _attach(core, "drone_1", 2.0)

    assert core.request_landing() is True
    snap = core.step(2.05)
    assert snap.state == "M2D_LANDING"
    assert snap.landing_vehicle_id == "drone_1"
    assert not any(snap.permissions.values())

    # Intentional release of the selected landing vehicle is not a fleet abort.
    core.update_phase("drone_1", "M2_LANDING_STAGE")
    core.update_attachment_evidence("drone_1", confirmed=False, lost=True, stamp_s=2.1)
    core.update_attachment_evidence("drone_0", confirmed=True, lost=False, stamp_s=2.1)
    snap = core.step(2.1)
    assert snap.aborted is False
    assert snap.landing_vehicle_id == "drone_1"

    core.update_phase("drone_1", "LANDED_DISARMED")
    core.update_attachment_evidence("drone_0", confirmed=True, lost=False, stamp_s=2.2)
    snap = core.step(2.2)
    assert snap.landing_vehicle_id == "drone_0"
    assert snap.landed_vehicle_ids == ("drone_1",)

    core.update_phase("drone_0", "M2_LANDING_STAGE")
    core.update_attachment_evidence("drone_0", confirmed=False, lost=True, stamp_s=2.3)
    snap = core.step(2.3)
    assert snap.aborted is False

    core.update_phase("drone_0", "LANDED_DISARMED")
    snap = core.step(2.4)
    assert snap.landed is True
    assert snap.landed_vehicle_ids == ("drone_0", "drone_1")
    assert snap.attached_plate_ids == {}


def test_m2d_operator_landing_keeps_other_attached_peers_protected():
    core = M2DSequentialSupervisorCore(
        evidence_timeout_s=0.5,
        target_attachment_count=2,
        operator_hold_after_goal=True,
    )
    core.freeze_assignment(ASSIGNMENT)
    core.update_m2c_ready(True)
    _attach(core, "drone_0", 1.0)
    _attach(core, "drone_1", 2.0)
    assert core.request_landing() is True

    core.update_phase("drone_1", "M2_LANDING_STAGE")
    core.update_attachment_evidence("drone_1", confirmed=False, lost=True, stamp_s=2.1)
    core.update_attachment_evidence("drone_0", confirmed=False, lost=True, stamp_s=2.1)
    snap = core.step(2.1)
    assert snap.aborted is True
    assert snap.abort_reason == "PROVEN_ATTACHMENT_LOST:drone_0"


def test_m2d_operator_landing_request_is_rejected_before_success_hold():
    core = M2DSequentialSupervisorCore(
        target_attachment_count=2,
        operator_hold_after_goal=True,
    )
    core.freeze_assignment(ASSIGNMENT)
    core.update_m2c_ready(True)
    assert core.snapshot().state == "ACTIVE_0"
    assert core.request_landing() is False
