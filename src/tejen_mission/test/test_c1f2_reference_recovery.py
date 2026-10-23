"""Interrupted-epoch recovery, without a ROS graph or flight."""

import importlib
import json
from types import SimpleNamespace
from pathlib import Path

import numpy as np
import pytest

from tejen_mission import c1f2_handoff as handoff
from tejen_mission.reference_generators import TrajectoryReference


def test_native_acceptance_is_not_caller_installation_or_a_new_stop_trigger():
    source = (Path(__file__).resolve().parents[3] /
              'src/tejen_dynamic_planner/src/transfer_backend_node.cpp').read_text()
    poll = source[source.index('    void pollPlannerFuture()'):source.index('    void replanTimer()')]
    replan = source[source.index('    void replanTimer()'):source.index('    void referenceTimer()')]
    reference = source[source.index('    void referenceTimer()'):source.index('    void referenceTimer()') + 6000]
    assert 'installCertifiedReplacement(' in poll
    assert 'planner_ = std::move(outcome.retained_planner)' in poll
    assert 'if (outcome.authority_installed)' in poll
    # Native non-replacement outcomes retain legitimate core bookkeeping (e.g.
    # measured GOAL_REACHED); only caller-rejected accepted shadows roll back.
    assert 'if (!outcome.worker_exception && !outcome.result.accepted) {' in poll
    assert replan.index('make_unique<RecedingHorizonPlanner>(*planner_)') < replan.index('auto planner = std::move(planner_)')
    assert 'execution_authority_certificate_' not in reference
    assert 'EXECUTION_CERTIFICATE_EXPIRED' not in source
    assert 'timing_policy' not in source  # no experimental profile activation


def test_pending_brake_installation_cannot_publish_or_launch_a_replacement():
    source = (Path(__file__).resolve().parents[3] /
              'src/tejen_dynamic_planner/src/transfer_backend_node.cpp').read_text()
    reference = source[source.index('    void referenceTimer()'):source.index('    void referenceTimer()') + 6000]
    replan = source[source.index('    void replanTimer()'):source.index('    void referenceTimer()')]
    assert 'ExecutionFallbackOutcome::NotInstalled' in reference
    assert 'if (!execution_fallback_pending_reason_.empty())' in replan
    assert replan.index('if (!execution_fallback_pending_reason_.empty())') < replan.index('auto planner = std::move(planner_)')
    activation = source[source.index('    ExecutionFallbackOutcome activateExecutionFallback('):source.index('    void updateExecutionTrackingDiagnostics(')]
    assert activation.index('++epoch_') < activation.index('attemptExecutionFallback(')
    assert 'execution_fallback_active_ = false' in activation
    disable = source[source.index('    void disableBackend('):source.index('    void publishRecoveryStatus(')]
    assert 'execution_fallback_pending_reason_.clear()' in disable
    assert 'cooperative blockers=%s' in disable
    assert 'cooperativeInputBlockers(' in disable


def hold(position=(.4, -.3, .9)):
    return TrajectoryReference(np.tile(position, (61, 1)),
                               np.zeros((61, 3)), np.zeros((61, 3)))


def recovery(reference=None):
    result = handoff.ReferenceRecovery()
    result.begin(request_id="a" * 32, reference=hold() if reference is None else reference,
                 now_s=10., ros_time_ns=1_000_000_000_000)
    return result


def status(result, *, epoch=3, prepared=False, granted=False, stamp_ns=0,
           request_id="a" * 32, now_s=10.1):
    result.observe_status(dict(request_id=request_id, epoch=epoch,
                               prepared=prepared, granted=granted,
                               reference_stamp_ns=stamp_ns, reason="ready"), now_s=now_s)


def accepts(result, reference=None, *, stamp_ns=1_000_100_000_000, now_s=10.2):
    return result.accept_reference(
        hold() if reference is None else reference, source_stamp_ns=stamp_ns,
        now_s=now_s, timeout_s=.2, position_tolerance_m=.03,
        velocity_tolerance_mps=.10, acceleration_tolerance_mps2=.50)


def test_common_seed_is_commanded_hold_not_measured_origin_and_requires_current_grant():
    result = recovery()
    assert " reset" in result.request(now_s=10., ros_time_ns=1_000_000_000_001, dt_s=1/30)
    status(result)
    seed = result.request(now_s=10.1, ros_time_ns=1_000_100_000_000, dt_s=1/30)
    assert " seed " in seed
    np.testing.assert_allclose([float(v) for v in seed.split()[2:5]], [.4, -.3, .9])
    status(result, prepared=True)
    assert " grant" in result.request(now_s=10.1, ros_time_ns=1_000_100_000_000, dt_s=1/30)
    assert not accepts(result)  # merely prepared is never executable authority
    status(result, granted=True, stamp_ns=1_000_100_000_000)
    assert accepts(result)
    assert not result.active


@pytest.mark.xfail(reason='multi_drone_control df62cd7: advance_reference_window pads past the window with a stopped '
                   'hold (the M2 join stall T0023); his recovery is off in every launch we run', strict=True)
def test_continuation_advances_without_inventing_a_stop_for_a_moving_window_tail():
    reference = hold()
    reference.positions[:, 0] += np.linspace(0., .1, 61)
    reference.velocities[:] = [.05, 0., 0.]
    result = recovery(reference)
    status(result)
    continued = result.reference(now_s=12.5, dt_s=1/30)
    np.testing.assert_allclose(continued.positions[0], [.5, -.3, .9])
    assert continued.velocities[0, 0] == .05
    assert " seed " not in (result.request(now_s=12.5, ros_time_ns=1_002_500_000_000, dt_s=1/30) or "")
    assert result.waiting_reason == "continuation_not_stationary"


def test_wrong_transaction_old_epoch_old_reference_and_stale_status_cannot_resume():
    result = recovery()
    status(result, granted=True, stamp_ns=1_000_100_000_000, request_id="b" * 32)
    assert not accepts(result)
    status(result, epoch=4)
    status(result, epoch=3, granted=True, stamp_ns=1_000_100_000_000)
    assert not accepts(result)
    status(result, epoch=4, granted=True, stamp_ns=999_000_000_000)
    assert not accepts(result, stamp_ns=999_000_000_000)
    status(result, epoch=4, granted=True, stamp_ns=1_000_100_000_000)
    assert not accepts(result, now_s=10.5)
    assert not accepts(result, stamp_ns=1_000_100_000_001)
    status(result, epoch=4, granted=True, stamp_ns=1_000_100_000_000, now_s=10.5)
    assert not accepts(result, now_s=10.55)  # renewed status cannot renew an old source
    status(result, epoch=4, granted=True, stamp_ns=1_000_530_000_000, now_s=10.54)
    assert accepts(result, stamp_ns=1_000_530_000_000, now_s=10.55)


def test_certified_but_discontinuous_reference_is_rejected_without_relaxing_pva():
    for field, delta in (("positions", .031), ("velocities", .101), ("accelerations", .501)):
        result = recovery()
        status(result, granted=True, stamp_ns=1_000_100_000_000)
        candidate = hold()
        getattr(candidate, field)[:, 0] += delta
        assert not accepts(result, candidate)
        assert result.active and result.waiting_reason == "reference_discontinuous"


def test_cancel_prevents_late_status_or_reference_from_reactivating_recovery():
    result = recovery()
    request = result.cancel()
    assert request == "a" * 32 + " cancel"
    status(result, granted=True, stamp_ns=1_001_000_000_000)
    assert not accepts(result)
    assert result.request(now_s=10.2, ros_time_ns=1_001_000_000_000, dt_s=1/30) is None


def test_negative_ack_enters_recovery_only_for_opted_in_cpp_transit():
    module = importlib.import_module("tejen_mission.online_join_planner")
    for enabled, phase, expected in (
        (True, module.MissionPhase.M2_CPP_TRANSIT_CAPTURE, True),
        (False, module.MissionPhase.M2_CPP_TRANSIT_CAPTURE, False),
        (True, module.MissionPhase.TRANSIT_TO_DROP_POINT, True),
        (False, module.MissionPhase.TRANSIT_TO_DROP_POINT, False),
        (True, module.MissionPhase.TAKEOFF, False),
        (True, module.MissionPhase.LANDING, False),
    ):
        planner = object.__new__(module.OnlineJoinPlanner)
        planner.c1f2_reference_recovery_enabled = enabled
        planner.c1f2_cpp_authority_enabled = True
        planner.c1f2_cpp_authority_active = True
        planner.c1f2_reference_recovery = handoff.ReferenceRecovery()
        planner.phase = phase
        planner._physical_now_s = lambda: 10.
        planner.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000_000))
        planner._c1f2_current_authoritative_reference = hold
        planner.c1f2_publish_authority = lambda value: None
        planner.c1f2_recovery_request_pub = SimpleNamespace(publish=lambda msg: None)
        planner.c1f2_recovery_request_count = 0
        planner.get_logger = lambda: SimpleNamespace(warning=lambda *a, **k: None)
        planner.c1f2_authority_ack_callback(SimpleNamespace(data=False))
        assert planner.c1f2_reference_recovery.active is expected


@pytest.mark.parametrize('phase_name', ['TRANSIT_TO_DROP_POINT', 'M2_CPP_TRANSIT_CAPTURE'])
@pytest.mark.parametrize('enabled', [False, True])
def test_late_negative_bool_ack_does_not_reset_a_current_granted_backend(phase_name, enabled):
    module = importlib.import_module("tejen_mission.online_join_planner")
    planner = object.__new__(module.OnlineJoinPlanner)
    planner.c1f2_reference_recovery_enabled = enabled
    planner.c1f2_cpp_authority_enabled = True
    planner.c1f2_cpp_authority_active = True
    planner.phase = getattr(module.MissionPhase, phase_name)
    planner.c1f2_recovery_backend_status = dict(epoch=2, granted=True, stamp_ns=1_001_000_000_000)
    planner.c1f2_recovery_last_accept_epoch = 2
    planner.c1f2_recovery_last_accept_stamp_ns = 1_001_000_000_000
    planner.c1f2_reference_recovery = handoff.ReferenceRecovery()
    calls = []
    planner.begin_c1f2_reference_recovery = calls.append
    planner._c1f2_try_finalize_authority_handoff = lambda: None
    planner._physical_now_s = lambda: 10.2
    planner.c1f2_authority_ack_callback(SimpleNamespace(data=False))
    assert calls == []
    planner.c1f2_recovery_status_callback(SimpleNamespace(data=json.dumps(dict(
        request_id="", epoch=3, prepared=False, granted=False, reference_stamp_ns=0,
        stamp_ns=1_002_000_000_000, reason="COOPERATIVE_SHARED_INPUT_INVALID_DISABLED"))))
    assert calls == (["backend_withdrew_authority"] if enabled else [])


def test_reference_status_pair_survives_cross_topic_reordering_and_expires():
    result = recovery()
    older_stamp, newer_stamp = 1_000_100_000_000, 1_000_133_333_333
    result.observe_reference(hold(), source_stamp_ns=older_stamp, receive_time_s=10.10)
    result.observe_reference(hold((.401, -.3, .9)), source_stamp_ns=newer_stamp,
                             receive_time_s=10.13)
    status(result, granted=True, stamp_ns=older_stamp, now_s=10.14)
    matched = result.matching_reference(now_s=10.15, timeout_s=.2)
    assert matched is not None
    reference, stamp, received = matched
    assert stamp == older_stamp and received == 10.10
    np.testing.assert_allclose(reference.positions[0], [.4, -.3, .9])
    assert result.matching_reference(now_s=10.31, timeout_s=.2) is None
    result.cancel()
    assert result.matching_reference(now_s=10.15, timeout_s=.2) is None


def test_real_stopped_tail_is_retained_beyond_the_two_second_mpc_window():
    # A six-second certified commitment, not a two-second moving endpoint.
    t = np.arange(181) / 30.
    s = t / 6.
    positions = np.tile([.4, -.3, .9], (len(t), 1))
    positions[:, 0] += .3 * (10*s**3 - 15*s**4 + 6*s**5)
    velocities = np.zeros_like(positions)
    accelerations = np.zeros_like(positions)
    velocities[:, 0] = .3/6 * (30*s**2 - 60*s**3 + 30*s**4)
    accelerations[:, 0] = .3/36 * (60*s - 180*s**2 + 120*s**3)
    full = TrajectoryReference(positions, velocities, accelerations)
    result = recovery(full)
    status(result)
    halfway = result.reference(now_s=13., dt_s=1/30)
    assert halfway.positions[0, 0] == .55
    assert halfway.velocities[0, 0] > 0
    assert result.request(now_s=13., ros_time_ns=1_003_000_000_000, dt_s=1/30) is None
    stopped = result.reference(now_s=16.1, dt_s=1/30)
    np.testing.assert_allclose(stopped.positions[:, 0], .7)
    assert ' seed ' in result.request(now_s=16.1, ros_time_ns=1_006_100_000_000, dt_s=1/30)


@pytest.mark.parametrize("transport_only", [False, True])
def test_opt_in_planner_retains_full_tail_and_advances_by_source_time(transport_only):
    module = importlib.import_module('tejen_mission.online_join_planner')
    p = object.__new__(module.OnlineJoinPlanner)
    p.c1f2_reference_recovery_enabled = not transport_only
    p.c1f2_stopped_continuation_enabled = True
    full = TrajectoryReference(np.tile([.4, -.3, .9], (181, 1)),
                               np.zeros((181, 3)), np.zeros((181, 3)))
    full.positions[:120, 0] = np.linspace(.2, .4, 120)
    p.c1f1_shadow_full_reference = full
    p.c1f1_shadow_reference_source_stamp_s = 100.
    p.c1f1_shadow_reference_last_receive_time = 100.09
    p.c1f1_shadow_reference_age_ms = lambda: 90.
    p._physical_now_s = lambda: 100.10
    p.dt = 1/30
    rolling = TrajectoryReference(full.positions[:61], full.velocities[:61], full.accelerations[:61])
    p._c1f2_accept_cpp_candidate(rolling)
    assert p.c1f2_cached_cpp_reference.positions.shape == (181, 3)
    assert p.c1f2_cached_cpp_reference_receive_time == 100.
    np.testing.assert_allclose(p._c1f2_advanced_cached_reference().positions[0], full.positions[3])
    p._physical_now_s = lambda: 105.
    stopped = p._c1f2_advanced_cached_reference()
    np.testing.assert_allclose(stopped.positions[:, 0], .4)
    np.testing.assert_allclose(stopped.velocities, 0.)


@pytest.mark.parametrize("transport_only", [False, True])
def test_opt_in_snapshot_uses_elapsed_physical_time_not_previous_timer_period(transport_only):
    module = importlib.import_module('tejen_mission.online_join_planner')
    p = object.__new__(module.OnlineJoinPlanner)
    p.c1f2_reference_recovery_enabled = not transport_only
    p.c1f2_stopped_continuation_enabled = True
    p.last_reference = hold()
    p.last_reference.positions[:, 0] += np.arange(61) / 300.
    p.last_reference_time_s = 100.
    p.last_planner_callback_period_ms = 250.
    p._physical_now_s = lambda: 100.01
    p.dt = 1/30
    current = p._c1f2_current_authoritative_reference()
    assert abs(current.positions[0, 0] - .401) < 1e-10


def parser_adapter(now=100.1, transport_only=False):
    module = importlib.import_module('tejen_mission.online_join_planner')
    p = object.__new__(module.OnlineJoinPlanner)
    p.c1f2_reference_recovery_enabled = not transport_only
    p.c1f2_stopped_continuation_enabled = True
    p.c1f1_shadow_transfer_enabled = True
    p.c1f1_shadow_reference_timeout_s = .2
    p.c1f1_shadow_reference_source_stamp_ns = None
    p.c1f1_shadow_reference_last_receive_time = None
    p.c1f1_shadow_reference_count = 0
    p.c1f1_shadow_reference_terminal = np.full(3, np.nan)
    p.c1f1_shadow_reference = p.c1f1_shadow_full_reference = None
    p.dt = 1/30
    p._physical_now_s = lambda: now
    p._c1f2_try_finalize_authority_handoff = lambda: None
    return p


def packet(reference, stamp_s):
    points = []
    for k, (pos, vel, acc) in enumerate(zip(reference.positions, reference.velocities,
                                           reference.accelerations)):
        vector = lambda x: SimpleNamespace(x=float(x[0]), y=float(x[1]), z=float(x[2]))
        ns = round(k/30 * 1e9)
        points.append(SimpleNamespace(transforms=[SimpleNamespace(translation=vector(pos))],
            velocities=[SimpleNamespace(linear=vector(vel))],
            accelerations=[SimpleNamespace(linear=vector(acc))],
            time_from_start=SimpleNamespace(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)))
    ns = round(stamp_s * 1e9)
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(
        sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)), points=points)


@pytest.mark.parametrize('phase_name', ['TRANSIT_TO_DROP_POINT', 'M2_CPP_TRANSIT_CAPTURE'])
def test_withdrawal_retains_real_stopped_tail_then_resumes_with_new_transaction(phase_name):
    module = importlib.import_module('tejen_mission.online_join_planner')
    p = parser_adapter()
    now = [100.1]
    p._physical_now_s = lambda: now[0]
    p.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(now[0]*1e9)))
    p.phase = getattr(module.MissionPhase, phase_name)
    p.c1f2_cpp_authority_enabled = p.c1f2_cpp_authority_active = True
    p.c1f2_initial_prepare_active = p.c1f2_prepare_phase_active = lambda: False
    p.c1f2_reference_recovery = handoff.ReferenceRecovery()
    p.c1f2_recovery_last_accept_epoch, p.c1f2_recovery_last_accept_stamp_ns = 2, 100_000_000_000
    p.c1f2_recovery_backend_status = dict(epoch=2, granted=True, stamp_ns=100_000_000_000)
    p.c1f2_recovery_request_count = p.c1f2_recovery_resume_count = 0
    p.c1f2_handoff_position_tolerance_m, p.c1f2_handoff_velocity_tolerance_mps = .03, .1
    p.c1f2_handoff_acceleration_tolerance_mps2 = .5
    requests, grants = [], []
    p.c1f2_recovery_request_pub = SimpleNamespace(publish=lambda msg: requests.append(msg.data))
    p.c1f2_publish_authority = grants.append
    p.get_logger = lambda: SimpleNamespace(info=lambda *a: None, warning=lambda *a, **k: None,
                                          error=lambda *a, **k: None)
    # A six-second continuation includes a real stopped tail beyond the MPC window.
    t = np.arange(181)/30
    s = t/6
    pos = np.tile([.4, -.3, .9], (len(t), 1))
    vel, acc = np.zeros_like(pos), np.zeros_like(pos)
    pos[:, 0] += .3*(10*s**3-15*s**4+6*s**5)
    vel[:, 0] = .3/6*(30*s**2-60*s**3+30*s**4)
    acc[:, 0] = .3/36*(60*s-180*s**2+120*s**3)
    full = TrajectoryReference(pos, vel, acc)
    p.c1f1_shadow_reference_callback(packet(full, 100.))
    p._c1f2_accept_cpp_candidate(p.c1f1_shadow_reference)
    p._c1f2_current_authoritative_reference = p._c1f2_advanced_cached_reference
    del p._c1f2_try_finalize_authority_handoff  # Exercise the real ROS adapter method.
    p.c1f2_recovery_status_callback(SimpleNamespace(data=json.dumps(dict(
        request_id='a'*32, epoch=3, stamp_ns=100_100_000_000, prepared=False,
        granted=False, reference_stamp_ns=0, reason='COOPERATIVE_SHARED_INPUT_INVALID_DISABLED'))))
    r = p.c1f2_reference_recovery
    assert r.active and r.request_id != 'a'*32
    assert requests == [r.request_id+' reset'] and grants == [False]
    p.c1f2_recovery_status_callback(SimpleNamespace(data=json.dumps(dict(
        request_id=r.request_id, epoch=4, stamp_ns=100_100_000_000, prepared=False,
        granted=False, reference_stamp_ns=0, reason='WAITING_FOR_VALID_COOPERATIVE_SHARED_TRAJECTORIES'))))
    now[0] = 103.
    continued = p.select_c1f2_authority_reference(None)
    expected = handoff.advance_reference_window(full, elapsed_s=3., sample_dt_s=p.dt, output_count=61)
    for field in ('positions', 'velocities', 'accelerations'):
        np.testing.assert_allclose(getattr(continued, field), getattr(expected, field), atol=1e-12)
    assert r.request(now_s=now[0], ros_time_ns=103_000_000_000, dt_s=p.dt) is None
    assert r.waiting_reason == 'continuation_not_stationary'
    now[0] = 106.1
    stopped = p.select_c1f2_authority_reference(None)
    np.testing.assert_allclose(stopped.positions[:, 0], .7)
    seed = r.request(now_s=now[0], ros_time_ns=106_100_000_000, dt_s=p.dt)
    assert ' seed ' in seed and ' grant ' not in seed
    np.testing.assert_allclose(list(map(float, seed.split()[2:5])), [.7, -.3, .9])
    # Valid-looking old initial-transaction evidence cannot resume this attempt.
    r.observe_status(dict(request_id='a'*32, epoch=99, prepared=True, granted=True,
                          reference_stamp_ns=106_110_000_000), now_s=106.11)
    assert r.status['granted'] is False
    now[0] = 106.12
    r.observe_status(dict(request_id=r.request_id, epoch=5, prepared=False, granted=True,
                          reference_stamp_ns=106_110_000_000), now_s=now[0])
    candidate = hold((.7, -.3, .9))
    r.observe_reference(candidate, source_stamp_ns=106_110_000_000, receive_time_s=now[0])
    p._c1f2_try_finalize_authority_handoff()
    assert r.active  # A rolling window alone cannot authorise the resumed full tail.
    p.c1f1_shadow_reference_callback(packet(candidate, 106.11))
    assert not r.active and p.c1f2_cpp_authority_active
    assert p.c1f2_recovery_resume_count == 1 and grants[-1] is True
    assert p.c1f2_recovery_last_accept_epoch == 5


@pytest.mark.parametrize("transport_only", [False, True])
def test_actual_parser_keeps_full_stopped_tail_and_does_not_refresh_on_invalid_packet(transport_only):
    p = parser_adapter(transport_only=transport_only)
    full = TrajectoryReference(np.tile([.4, -.3, .9], (181, 1)),
                               np.zeros((181, 3)), np.zeros((181, 3)))
    p.c1f1_shadow_reference_callback(packet(full, 100.))
    assert len(p.c1f1_shadow_reference.positions) == 61
    assert len(p.c1f1_shadow_full_reference.positions) == 181
    accepted_stamp = p.c1f1_shadow_reference_source_stamp_ns
    accepted_time = p.c1f1_shadow_reference_last_receive_time
    bad = packet(full, 100.05)
    bad.points[7].time_from_start.nanosec += 10000
    p._physical_now_s = lambda: 100.15
    p.c1f1_shadow_reference_callback(bad)
    assert p.c1f1_shadow_reference_source_stamp_ns == accepted_stamp
    assert p.c1f1_shadow_reference_last_receive_time == accepted_time
    # Rejection must not overwrite the usable committed tail or its identity.
    assert p.c1f1_shadow_full_reference is full or len(p.c1f1_shadow_full_reference.positions) == 181
    for stamp in (99., 100., 101.):
        p.c1f1_shadow_reference_callback(packet(full, stamp))
    assert p.c1f1_shadow_reference_last_receive_time == accepted_time


@pytest.mark.parametrize("transport_only", [False, True])
def test_delayed_fresh_packet_and_cached_continuation_have_identical_timebase(transport_only):
    module = importlib.import_module('tejen_mission.online_join_planner')
    p = parser_adapter(transport_only=transport_only)
    full = TrajectoryReference(np.tile([.4, -.3, .9], (181, 1)),
                               np.zeros((181, 3)), np.zeros((181, 3)))
    full.positions[:, 0] += np.minimum(np.arange(181)/300., .4)
    full.velocities[:120, 0] = .1
    p.c1f1_shadow_reference_callback(packet(full, 100.))
    p.c1f2_cpp_authority_enabled = p.c1f2_cpp_authority_active = True
    p.phase = (module.MissionPhase.TRANSIT_TO_DROP_POINT if transport_only else
               module.MissionPhase.M2_CPP_TRANSIT_CAPTURE)
    p.c1f2_prepare_phase_active = lambda: False
    p.c1f2_reference_recovery = handoff.ReferenceRecovery()
    p.c1f2_cached_continuation_count = 0
    p.get_logger = lambda: SimpleNamespace(warning=lambda *a: None)
    p._c1f2_accept_cpp_candidate(p.c1f1_shadow_reference)
    selected = p.select_c1f2_authority_reference(None)
    cached = p._c1f2_advanced_cached_reference()
    for field in ('positions', 'velocities', 'accelerations'):
        np.testing.assert_allclose(getattr(selected, field), getattr(cached, field))


def test_receipt_freshness_cannot_renew_an_old_recovery_source_publication():
    r = handoff.ReferenceRecovery()
    r.begin(request_id='a'*32, reference=hold(), now_s=100., ros_time_ns=100_000_000_000)
    r.reference(now_s=100.1, dt_s=1/30)
    r.observe_reference(hold(), source_stamp_ns=100_010_000_000, receive_time_s=100.20)
    status(r, stamp_ns=100_010_000_000, granted=True, now_s=100.38)
    assert r.matching_reference(now_s=100.38, timeout_s=.2) is None


@pytest.mark.parametrize('phase_name', ['TRANSIT_TO_DROP_POINT', 'M2_CPP_TRANSIT_CAPTURE'])
def test_land_callback_cancels_recovery_before_waiting_for_pose(phase_name):
    module = importlib.import_module('tejen_mission.online_join_planner')
    p = object.__new__(module.OnlineJoinPlanner)
    p.c1f2_reference_recovery = recovery()
    p.phase = getattr(module.MissionPhase, phase_name)
    p.ground_ring_exit = None
    p.is_m2b = p.m2b_progressive = phase_name == 'M2_CPP_TRANSIT_CAPTURE'
    p.drone_position = None
    p.get_logger = lambda: SimpleNamespace(warning=lambda *a: None)
    sent = []
    p.c1f2_recovery_request_pub = SimpleNamespace(publish=lambda msg: sent.append(msg.data))
    p.external_landing_callback(SimpleNamespace(data=True))
    assert sent == ['a'*32 + ' cancel']
    assert not p.c1f2_reference_recovery.active
    assert p.external_landing_pending
