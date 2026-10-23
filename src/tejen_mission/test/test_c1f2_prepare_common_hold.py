"""Initial handoff uses the commanded hold, not a repeatedly measured origin."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from tejen_mission.c1f2_handoff import ReferenceRecovery
from tejen_mission.online_join_planner import OnlineJoinPlanner, MissionPhase
from tejen_mission.reference_generators import TargetState, TrajectoryReference


def hold(z=1.224):
    return TrajectoryReference(np.tile([-.4, -.3, z], (61, 1)),
                               np.zeros((61, 3)), np.zeros((61, 3)))


def planner(phase):
    p = object.__new__(OnlineJoinPlanner)
    p.phase = phase
    p.preloaded_navigation = phase == MissionPhase.TAKEOFF
    p.experiment_start_requested = True
    p.is_m2b = phase == MissionPhase.M2_TAKEOFF_HOVER
    p.m2b_commissioning_stage = 'b2'
    p.c1f1_shadow_transfer_enabled = p.c1f2_cpp_authority_enabled = True
    p.c1f2_prepare_active = p.c1f2_prepare_settled = True
    p.c1f2_prepare_lift_end_position = np.array([-.4, -.3, 1.26])
    p.c1f2_prepare_hold_position = hold().positions[0].copy()
    p.drone_position = np.array([-.4, -.3, 1.178])
    p.drone_velocity = p.drone_acceleration = np.zeros(3)
    p.c1f2_prepare_max_lift_drift_m = .15
    p.c1f2_prepare_settle_speed_mps = .08
    p.c1f2_prepare_settle_acceleration_mps2 = .5
    p.c1f2_handoff_position_tolerance_m = .03
    p.c1f2_handoff_velocity_tolerance_mps = .1
    p.c1f2_handoff_acceleration_tolerance_mps2 = .5
    p.c1f2_handoff_ready = p.c1f2_cpp_authority_active = False
    p.c1f2_authority_grant_pending = p.c1f2_commanded_authority = False
    p.c1f2_authority_acknowledged = False
    p.c1f2_authority_grant_count = p.c1f2_recovery_request_count = p.c1f2_recovery_resume_count = 0
    p.c1f2_reference_recovery = ReferenceRecovery()
    p.c1f2_recovery_backend_status = None
    p.c1f1_shadow_reference = hold(1.178)  # Passive seed, not an accepted plan.
    p.c1f1_shadow_full_reference = None
    p.c1f1_shadow_reference_sample_count = 61
    p.c1f1_shadow_reference_fresh = lambda: True
    p.c1f1_shadow_reference_last_receive_time = 100.
    p.c1f1_shadow_reference_source_stamp_ns = 100_000_000_000
    p.c1f1_shadow_reference_timeout_s = .2
    p.c1f1_shadow_commanded_enable = False
    p.c1f2_reference_recovery_enabled = p.c1f2_stopped_continuation_enabled = False
    p.horizon_samples, p.min_reference_z, p.dt = 61, .4, 1/30
    p._physical_now_s = lambda: 100.
    p.get_clock = lambda: NS(now=lambda: NS(nanoseconds=int(p._physical_now_s()*1e9)))
    p._c1f2_current_authoritative_reference = hold
    p.get_logger = lambda: NS(info=lambda *a, **k: None, warning=lambda *a, **k: None)
    p.requests, p.grants, p.enables = [], [], []
    p.c1f2_recovery_request_pub = NS(publish=lambda msg: p.requests.append(msg.data))
    p.c1f2_publish_authority = p.grants.append
    p.c1f1_publish_shadow_enable = p.enables.append
    p.c1f1_shadow_phase_active = lambda: False
    p.c1f2_drop_target_inputs_ready = lambda: True
    p.c1f2_transfer_target_state = lambda: TargetState(np.ones(3), np.zeros(3), np.zeros(3))
    p.c1f1_publish_shadow_target = p.c1f2_publish_target_velocity = lambda value: None
    return p


@pytest.mark.parametrize('phase', [MissionPhase.LIFT_OBJECT, MissionPhase.TAKEOFF,
                                   MissionPhase.M2_TAKEOFF_HOVER])
def test_passive_seed_and_failed_searches_never_walk_the_hold_or_request_authority(phase):
    p = planner(phase)
    out = p.select_c1f2_authority_reference(hold())
    np.testing.assert_array_equal(out.positions, hold().positions)
    assert p.c1f2_reference_recovery.active
    transaction = p.c1f2_reference_recovery.request_id
    for tick in range(1, 51):
        now = 100.+tick*.1
        p._physical_now_s = lambda: now
        p.c1f2_reference_recovery.observe_status(dict(request_id=transaction, epoch=3,
            prepared=False, granted=False, reference_stamp_ns=int(now*1e9),
            reason='LOCAL_PLAN_FAILED'), now_s=now)
        p.update_c1f1_shadow_transfer()
        out = p.select_c1f2_authority_reference(hold())
        np.testing.assert_array_equal(out.positions, hold().positions)
    assert not p.c1f2_authority_grant_pending
    assert True not in p.grants
    assert all(' grant ' not in message for message in p.requests)
    seed = p.requests[-1].split()
    assert seed[1] == 'seed'
    np.testing.assert_array_equal(list(map(float, seed[2:5])), hold().positions[0])


def test_prepare_entry_continues_previous_pva_before_using_its_stopped_endpoint():
    p = planner(MissionPhase.TAKEOFF)
    previous = hold()
    t = np.arange(61)/30
    s = t/2
    previous.positions[:, 2] += .1*(10*s**3-15*s**4+6*s**5)
    previous.velocities[:, 2] = .05*(30*s**2-60*s**3+30*s**4)
    previous.accelerations[:, 2] = .025*(60*s-180*s**2+120*s**3)
    p._c1f2_current_authoritative_reference = lambda: previous
    out = p.select_c1f2_authority_reference(hold(1.178))
    for field in ('positions', 'velocities', 'accelerations'):
        np.testing.assert_array_equal(getattr(out, field), getattr(previous, field))
    r = p.c1f2_reference_recovery
    r.observe_status(dict(request_id=r.request_id, epoch=3, prepared=False, granted=False,
                         reference_stamp_ns=0), now_s=100.)
    assert r.request(now_s=100., ros_time_ns=100_000_000_000, dt_s=p.dt) is None
    p._physical_now_s = lambda: 102.1
    out = p.select_c1f2_authority_reference(hold(1.178))
    np.testing.assert_allclose(out.positions[:, 2], 1.324)
    assert r.hold_position[2] == 1.324


def test_real_prepared_status_and_fresh_granted_reference_complete_initial_handoff():
    p = planner(MissionPhase.LIFT_OBJECT)
    p.select_c1f2_authority_reference(hold())
    r = p.c1f2_reference_recovery
    r.observe_status(dict(request_id=r.request_id, epoch=3, prepared=True, granted=False,
                         reference_stamp_ns=100_010_000_000), now_s=100.01)
    p._physical_now_s = lambda: 100.02
    p.update_c1f1_shadow_transfer()
    assert ' grant ' in p.requests[-1]
    assert not p.c1f2_handoff_ready
    p.c1f1_shadow_reference = hold()
    p.c1f1_shadow_reference_source_stamp_ns = 100_030_000_000
    p.c1f1_shadow_reference_last_receive_time = 100.03
    p._physical_now_s = lambda: 100.04
    r.observe_status(dict(request_id=r.request_id, epoch=3, prepared=False, granted=True,
                         reference_stamp_ns=100_030_000_000), now_s=100.04)
    p._c1f2_try_finalize_authority_handoff()
    assert p.c1f2_handoff_ready and not r.active
    assert not p.c1f2_cpp_authority_active  # Actual mission transition still owns the switch.


def test_motion_during_pending_initial_grant_cancels_without_moving_the_reference():
    p = planner(MissionPhase.TAKEOFF)
    p.select_c1f2_authority_reference(hold())
    p.c1f2_authority_grant_pending = True
    p.drone_velocity = np.array([0., 0., -.107])
    p.drone_acceleration = np.array([0., 0., -.527])
    out = p.select_c1f2_authority_reference(hold())
    np.testing.assert_array_equal(out.positions, hold().positions)
    assert not p.c1f2_prepare_settled and not p.c1f2_authority_grant_pending
    assert any(message.endswith(' cancel') for message in p.requests)


def test_backend_exposes_existing_common_seed_transport_without_enabling_inflight_policy():
    from pathlib import Path
    source = (Path(__file__).resolve().parents[3]/
              'src/tejen_dynamic_planner/src/transfer_backend_node.cpp').read_text()
    constructor = source.split('    void publishRecoveryStatus(', 1)[0]
    transport = constructor.split('        authority_ack_pub_ =', 1)[1].split('        rclcpp::QoS marker_qos', 1)[0]
    assert 'if (reference_recovery_enabled_)' not in transport
    assert 'declare_parameter<bool>("reference_recovery_enabled", false)' in source
