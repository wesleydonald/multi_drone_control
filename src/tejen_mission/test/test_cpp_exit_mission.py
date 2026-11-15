"""The C++ exit's mission side (docs/rig/cpp_exit_impl_contract_2026-11-15.md sections 3-4), driven without ROS through
the planner tick's own order (state machine, Python reference, backend topics, selection, publication): the clearance
keeps following the moving ring and starts the 7 s budget; the prefix is published and executed as one curve; the
transaction is reset, seedtraj per prefix, grant; a granted window joins only on the prefix; timeout and rejection
cancel back to the live reference without a step; LAND between two publications selects Python first, cancels, and
starts the escape at the true publication time; AT_START lands after its dwell; and with the switch off nothing
changes (against the source as committed)."""
import importlib.util
import itertools
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from tejen_mission import c1f2_handoff as h
from tejen_mission import online_join_planner as ojp
from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.reference_generators import TrajectoryReference, TransferConfig, VirtualTransferGenerator
from test_floor_join_delivery_rejoin import place
from test_m1_contact_attach import QUAD_FROM_TIP, Recorder
from test_return_after_drop import START, returning

T0 = 1_763_200_000.0
DT = 1.0 / 30.0
RING0 = np.array([0.3, -0.2, 1.0])
V_PLAN = np.array([0.10, 0.05, 0.0])      # the ring's committed future
V_RING = np.array([0.12, 0.05, 0.0])      # the measured ring (2 cm/s off its plan)
LIMITS = dict(c1f2_bridge_max_velocity=np.ones(3), c1f2_bridge_max_acceleration=np.array([1.0, 1.0, 1.5]),
              c1f2_bridge_max_jerk=np.full(3, 4.0))


class Snapshots(Recorder):
    """A publisher that also records the planner's selection flags at each publication."""

    def __init__(self, planner):
        super().__init__()
        self.planner, self.flags = planner, []

    def publish(self, msg):
        p = self.planner
        self.flags.append((p.c1f2_cpp_authority_active, p.exit_escape_active,
                           p.exit_handoff.active if p.exit_handoff else None))
        super().publish(msg)


def ring_commitment(t0=T0 - 1.0, t1=T0 + 40.0):
    """A ring_bridge-style commitment: Greville control points of the planned straight path (exact for a line)."""
    knots = h.open_uniform_knots(t0, t1, int(round((t1 - t0) / 0.5)))
    greville = (knots[1:-3] + knots[2:-2] + knots[3:-1]) / 3.0
    cps = RING0 + np.outer(greville - T0, V_PLAN)
    return h.CommittedSpline([h.SplinePiece(knots, cps)])


def stub_clock(p):
    def now():
        ns = int(round(p.clock.t * 1e9))
        return NS(nanoseconds=ns, to_msg=lambda: Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000))
    p.get_clock = lambda: NS(now=now)


def exit_planner(switch=True, cls_planner=None):
    p = cls_planner if cls_planner is not None else returning('cpp')
    p.contact_attach = False
    p.clock.t = T0
    stub_clock(p)
    p.dt, p.horizon_samples = DT, 61
    p.min_reference_z, p.max_reference_speed = 0.0, 2.0
    if switch is not None:
        p.c1f2_exit_moving_handoff = switch
        p.exit_handoff = h.MovingExitHandoff() if switch else None
        p.exit_prefix_horizon_s, p.exit_prefix_refresh_s, p.exit_blend_s = 5.0, 1.0, 1.0
        p.exit_protected_s, p.exit_handoff_timeout_s, p.exit_hold_z_m = 1.5, 7.0, 2.9
        p.exit_ring_commitment_max_age_s, p.at_start_land_after_s = 2.0, 0.0
        p.exit_blend, p.exit_escape_active, p.exit_climb_tip_z = None, False, None
        p.exit_reference_stamp_ns, p.exit_last_source = None, 'other'
        p.exit_ring_commitment, p.exit_ring_commitment_sequence = ring_commitment(), 1
        p.exit_ring_commitment_receive_s = T0
        p.exit_land_epoch = p.exit_land_sequence = None
        p.last_reference_stamp_ns = None
    for k, v in LIMITS.items():
        setattr(p, k, v.copy())
    p.c1f2_bridge_feasibility_sample_dt_s = 0.02
    p.c1f2_prepare_max_lift_drift_m, p.c1f2_prepare_settle_speed_mps = 0.30, 0.12
    p.c1f2_prepare_settle_acceleration_mps2, p.c1f2_prepare_revoke_speed_mps = 0.50, 0.12
    p.c1f2_prepare_settle_dwell_s = 0.30
    p.c1f2_handoff_position_tolerance_m, p.c1f2_handoff_velocity_tolerance_mps = 0.03, 0.10
    p.c1f2_handoff_acceleration_tolerance_mps2 = 0.50
    p.drop_point_source, p.drop_offset, p.payload_yaw = 'payload', np.array([0.0, 0.0, 0.15]), 0.0
    p.drop_release_clearance, p.drop_clear_height, p.drop_clear_speed = 0.03, 0.06, 0.5
    p.drop_approach_clearance, p.drop_hold_offset = 0.5, None
    p.frame_id, p.vehicle_id = 'map', 'drone_0'
    p.landing_descent_speed = 0.25
    p.m2d_enabled, p.m2b_initial_yaw = False, None
    p.c1f2_reference_recovery = h.ReferenceRecovery()
    p.c1f2_recovery_backend_status = None
    p.c1f2_recovery_last_accept_epoch, p.c1f2_recovery_last_accept_stamp_ns = -1, 0
    p.c1f2_stopped_continuation_enabled = p.c1f2_reference_recovery_enabled = False
    p.c1f1_shadow_reference = p.c1f1_shadow_full_reference = None
    p.c1f1_shadow_reference_last_receive_time = p.c1f1_shadow_reference_source_stamp_ns = None
    p.c1f1_shadow_reference_source_stamp_s, p.c1f1_shadow_reference_sample_count = None, 0
    p.c1f1_shadow_reference_timeout_s = 0.2
    p.c1f2_authority_switch_count = p.c1f2_authority_grant_count = 0
    p.c1f2_cached_continuation_count = p.c1f2_recovery_reject_count = 0
    p.c1f2_recovery_request_count = p.c1f2_recovery_resume_count = 0
    p.c1f2_exit_bridge_active = False
    p.last_reference = p.last_reference_time_s = None
    p.last_planner_callback_period_ms = float('nan')
    p.transfer_generator = VirtualTransferGenerator(TransferConfig(
        dt=DT, horizon_samples=61, nominal_speed=0.4, duration_scale=1.2, min_duration=1.0, max_duration=30.0,
        tracking_error_soft=0.2, tracking_error_hard=0.6, minimum_progress_scale=0.2, min_reference_z=0.0,
        max_reference_speed=2.0))
    for name in ('c1f2_recovery_request_pub', 'c1f1_shadow_enable_publisher', 'c1f2_authority_publisher',
                 'reference_publisher', 'exit_prefix_publisher', 'c1f1_shadow_target_publisher',
                 'c1f2_target_velocity_publisher', 'c1f2_static_target_publisher'):
        setattr(p, name, Snapshots(p) if switch else Recorder())
    p.c1f1_shadow_commanded_enable = False
    p.phase = P.CLEAR_DROP_ZONE
    p.phase_entry_time = T0 - 2.15          # the 0.5 m/s climb ends ~0.1 s in
    ring_at(p, T0)
    body = p.build_reference().positions[0]
    place(p, body, V_RING)
    return p


def ring_at(p, t):
    p.payload_position = RING0 + V_RING * (t - T0)
    p.payload_velocity = V_RING.copy()
    p.last_payload_pose_time = p.last_payload_twist_time = t


def published_state(p, t):
    """The last published horizon at t (its samples advanced from their stamp)."""
    stamp = p.last_reference_stamp_ns * 1e-9 if getattr(p, 'last_reference_stamp_ns', None) else p.last_reference_time_s
    w = h.advance_reference_window(p.last_reference, elapsed_s=max(0.0, t - stamp), sample_dt_s=DT, output_count=1)
    return w.positions[0], w.velocities[0], w.accelerations[0]


def tick(p, dt=DT, refresh_ring=True):
    """One planner tick in timer_callback's order; QUAD4 flies the published reference exactly."""
    p.clock.t += dt
    ring_at(p, p.clock.t)
    if refresh_ring and getattr(p, 'exit_ring_commitment', None) is not None:
        p.exit_ring_commitment_receive_s = p.clock.t
    if p.last_reference is not None:
        pos, vel, acc = published_state(p, p.clock.t)
        place(p, pos, vel)
        p.drone_acceleration = acc.copy()
    p.update_state_machine()
    python = None if p.c1f2_cpp_transit_authority_active() else p.build_reference()
    p.update_c1f1_shadow_transfer()
    ref = p.select_c1f2_authority_reference(python)
    p.last_reference, p.last_reference_time_s = ref, p.clock.t
    p.publish_reference(ref)
    return ref, python


def requests(p):
    return [m.data for m in p.c1f2_recovery_request_pub.messages]


def status(p, **kw):
    tx = p.exit_handoff
    msg = {'request_id': tx.request_id, 'epoch': 5, 'stamp_ns': int(round(p.clock.t * 1e9)), 'prepared': False,
           'granted': False, 'reference_stamp_ns': 0, 'reason': '', 'exit_reject_reason': '',
           'prefix_sequence': tx.sequence, **kw}
    p.c1f2_recovery_status_callback(NS(data=json.dumps(msg)))


def cpp_window(p, stamp_s, join_s, shift_s=0.0):
    """The backend's granted window: the prefix up to its splice, his plan after it."""
    stamp_ns = int(round(stamp_s * 1e9))
    times = stamp_ns * 1e-9 + np.arange(61) * DT
    pos, vel, acc, _ = p.exit_handoff.prefix.state(np.minimum(times + shift_s, p.exit_handoff.prefix.end))
    pos = pos.copy()
    pos[times > join_s] += [0.0, 0.0, 0.02]
    p.c1f1_shadow_reference = TrajectoryReference(pos, vel, acc)
    p.c1f1_shadow_reference_source_stamp_ns, p.c1f1_shadow_reference_source_stamp_s = stamp_ns, stamp_ns * 1e-9
    p.c1f1_shadow_reference_last_receive_time = p.clock.t
    p.c1f1_shadow_reference_sample_count = 61
    p._c1f2_try_finalize_authority_handoff()
    return stamp_ns


def to_prefix(p):
    """Tick until the clearance completed and prefix 1 is published."""
    for _ in range(30):
        tick(p)
        if p.exit_handoff.sequence:
            return
    raise AssertionError('no prefix')


def to_granted(p):
    to_prefix(p)
    tick(p)                                         # reset + seedtraj 1
    status(p, prepared=True)
    tick(p)                                         # grant
    stamp = cpp_window(p, p.clock.t - 0.002, p.clock.t + 0.3)
    status(p, prepared=True, granted=True, reference_stamp_ns=stamp, splice_time_ns=int(round((p.clock.t + 0.3) * 1e9)))
    assert p.exit_handoff.state == 'accepted'
    tick(p)
    assert p.phase == P.RETURN_TO_START and p.c1f2_cpp_authority_active


def state_of(ref, i=0):
    return ref.positions[i], ref.velocities[i], ref.accelerations[i]


def close(a, b, tol):
    return max(float(np.max(np.abs(np.asarray(x) - np.asarray(y)))) for x, y in zip(a, b)) <= tol


# ---- the clearance, the timer and the prefix --------------------------------------------------------------------------

def test_the_clearance_follows_the_ring_to_the_hold_height_and_never_latches_a_world_point():
    p = exit_planner()
    tick(p)
    assert p.exit_handoff.state == 'idle'
    to_prefix(p)
    assert p.return_prepare_started and not p.c1f2_prepare_active and p.c1f2_prepare_hold_position is None
    # the live clearance reference: the ring's velocity, at the hold height
    _, live = tick(p)
    np.testing.assert_allclose(live.velocities[0], V_RING, atol=1e-12)
    assert live.positions[0][2] == pytest.approx(2.9, abs=1e-9)
    np.testing.assert_allclose(p.return_hover, [START[0], START[1], p.return_hover[2]])


def test_the_budget_starts_at_the_clearance_and_nothing_moves_it():
    p = exit_planner()
    p.exit_ring_commitment = None                   # no ring future yet: no prefix, no transaction
    for _ in range(12):
        tick(p)
    tx = p.exit_handoff
    assert tx.state == 'preparing' and tx.sequence == 0 and not tx.active
    t0, deadline = tx.t0_s, tx.deadline_ns
    assert abs(deadline - (int(round(t0 * 1e9)) + 7_000_000_000)) < 1_000         # t0 + 7 s (float t0)
    p.exit_ring_commitment = ring_commitment(T0 - 1.0, T0 + 40.0)
    tick(p)
    assert tx.sequence == 1
    for _ in range(int(2.5 / DT)):                  # two refreshes
        tick(p)
    assert tx.sequence == 3 and (tx.t0_s, tx.deadline_ns) == (t0, deadline)
    sent = [r for r in requests(p) if ' seedtraj ' in r]
    assert [int(r.split()[3]) for r in sent] == [1, 2, 3]
    assert {int(r.split()[5]) for r in sent} == {deadline}


def xyz(vectors):
    return np.array([[v.x, v.y, v.z] for v in vectors])


def test_published_samples_are_the_fitted_coefficients_at_their_absolute_times():
    p = exit_planner()
    to_prefix(p)
    for _ in range(int(1.2 / DT)):                  # across a refresh
        tick(p)
    tx = p.exit_handoff
    assert tx.sequence == 2
    msg = p.reference_publisher.messages[-1]
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    assert stamp_ns == int(round(p.clock.t * 1e9))
    pos, vel, acc, _ = tx.prefix.state(stamp_ns * 1e-9 + np.arange(61) * DT)
    np.testing.assert_array_equal(xyz(pt.transforms[0].translation for pt in msg.points), pos)
    np.testing.assert_array_equal(xyz(pt.velocities[0].linear for pt in msg.points), vel)
    np.testing.assert_array_equal(xyz(pt.accelerations[0].linear for pt in msg.points), acc)
    offsets = [pt.time_from_start.sec + 1e-9 * pt.time_from_start.nanosec for pt in msg.points]
    np.testing.assert_allclose(offsets, np.arange(61) * DT, atol=1e-9)


def test_the_prefix_message_starts_on_the_executed_reference():
    p = exit_planner()
    for _ in range(30):
        before = (p.last_reference, p.last_reference_stamp_ns)
        tick(p)
        if p.exit_handoff.sequence:
            break
    msg = p.exit_prefix_publisher.messages[-1]
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    ref_msg = p.reference_publisher.messages[-1]
    assert stamp_ns == ref_msg.header.stamp.sec * 1_000_000_000 + ref_msg.header.stamp.nanosec
    assert (msg.header.frame_id, msg.vehicle_id, msg.sequence, msg.terminal_hold) == ('map', 'drone_0', 1, False)
    spline = h.CommittedSpline.from_message(msg)
    assert spline.start == stamp_ns * 1e-9 and spline.end == pytest.approx(stamp_ns * 1e-9 + 5.0, abs=1e-6)
    window = h.advance_reference_window(before[0], elapsed_s=stamp_ns * 1e-9 - before[1] * 1e-9, sample_dt_s=DT,
                                        output_count=1)
    assert close([x[0] for x in spline.state([spline.start])[:3]], state_of(window), 1e-9)
    assert p.exit_handoff.offset is not None


def test_the_transaction_reset_seedtraj_on_the_next_tick_then_grant_on_relative_readiness():
    p = exit_planner()
    to_prefix(p)
    assert requests(p) == []                        # the prefix goes out first
    tick(p)
    tx = p.exit_handoff
    assert requests(p) == [f'{tx.request_id} reset', f'{tx.request_id} seedtraj exit 1 {tx.prefix_stamp_ns} '
                                                      f'{tx.deadline_ns}']
    header = p.exit_prefix_publisher.messages[-1].header.stamp
    assert tx.prefix_stamp_ns == header.sec * 1_000_000_000 + header.nanosec      # the backend binds on equality
    assert not [m for m in p.c1f1_shadow_enable_publisher.messages if m.data]     # the seedtraj enables
    status(p, prepared=True)
    p.drone_position = p.drone_position + [0.5, 0.0, 0.0]                         # off the reference: no grant
    p.update_c1f1_shadow_transfer()
    assert not [r for r in requests(p) if ' grant ' in r]
    tick(p)                                         # moving with the reference, no stop needed
    grants = [r for r in requests(p) if ' grant ' in r]
    assert grants and float(np.linalg.norm(p.drone_velocity)) > 0.1
    assert int(grants[-1].split()[2]) >= tx.prefix_stamp_ns


def test_a_granted_window_joins_only_on_the_prefix_then_the_cpp_return_flies():
    p = exit_planner()
    to_prefix(p)
    tick(p)
    status(p, prepared=True)
    tick(p)
    late = cpp_window(p, p.clock.t - 0.002, p.clock.t + 0.3, shift_s=0.03)          # a clock-shifted copy: refused
    status(p, prepared=True, granted=True, reference_stamp_ns=late, splice_time_ns=int(round((p.clock.t + 0.3) * 1e9)))
    assert p.exit_handoff.state == 'preparing' and p.exit_handoff.waiting_reason == 'reference_not_on_prefix'
    p.clock.t += 0.005
    stamp = cpp_window(p, p.clock.t - 0.002, p.clock.t + 0.3)
    status(p, prepared=True, granted=True, reference_stamp_ns=stamp, splice_time_ns=int(round((p.clock.t + 0.3) * 1e9)))
    assert p.exit_handoff.state == 'accepted' and p.c1f2_handoff_ready
    assert p.c1f2_authority_publisher.messages[-1].data is True
    tick(p)
    assert p.phase == P.RETURN_TO_START and p.c1f2_cpp_authority_active and p.exit_handoff.state == 'granted'


def test_no_refresh_once_his_backend_reports_the_grant():
    p = exit_planner()
    to_prefix(p)
    tick(p)
    status(p, prepared=True, granted=True)
    for _ in range(int(1.5 / DT)):
        tick(p)
    assert p.exit_handoff.sequence == 1
    status(p, prepared=True, granted=False)                 # a reverted grant: refreshes and grants resume
    tick(p)
    assert p.exit_handoff.sequence == 2 and ' grant ' in requests(p)[-1]


# ---- timeout, rejection, refresh failure -----------------------------------------------------------------------------

def assert_blends_to_live(p, switch_ref, python, executed):
    assert close(state_of(switch_ref), executed, 1e-6)          # no step at the switch
    for _ in range(int(1.6 / DT)):
        ref, python = tick(p)
    np.testing.assert_allclose(ref.positions, python.positions, atol=1e-12)     # then the live reference itself
    assert p.exit_blend is None


@pytest.mark.parametrize('cause', ['timeout', 'rejection', 'stale ring'])
def test_failure_cancels_and_blends_back_to_the_live_reference(cause):
    p = exit_planner()
    to_prefix(p)
    tick(p)
    tx = p.exit_handoff
    if cause == 'timeout':
        while p.clock.t + DT < tx.deadline_s:
            tick(p)
    elif cause == 'rejection':
        status(p, exit_reject_reason='object_attached_or_stale')
    else:
        p.exit_ring_commitment_receive_s = p.clock.t - 3.0
        while p.clock.t + DT < tx.last_refresh_s + 1.0:
            tick(p, refresh_ring=False)
    executed = h.CommittedSpline(tx.prefix.pieces).state([p.clock.t + DT])
    ref, python = tick(p, refresh_ring=cause != 'stale ring')
    assert tx.state == 'failed' and requests(p)[-1] == f'{tx.request_id} cancel'
    assert p.c1f1_shadow_enable_publisher.messages[-1].data is False
    assert p.phase == P.CLEAR_DROP_ZONE and p.exit_blend is not None and p.exit_blend['kind'] == 'cancel'
    assert_blends_to_live(p, ref, python, [x[0] for x in executed[:3]])
    assert p.phase == P.CLEAR_DROP_ZONE                          # no Python return, no world-fixed stop


def test_a_cancel_blend_that_fails_its_checks_flies_the_live_reference_as_it_is():
    p = exit_planner()
    to_prefix(p)
    tick(p)
    p.c1f2_bridge_max_jerk = np.full(3, 1e-4)
    status(p, exit_reject_reason='prefix_invalid')
    ref, python = tick(p)
    assert p.exit_handoff.state == 'failed' and p.exit_blend is None
    np.testing.assert_array_equal(ref.positions, python.positions)


# ---- manual LAND -----------------------------------------------------------------------------------------------------

@pytest.mark.parametrize('when', ['preparing', 'after the grant'])
def test_land_between_two_publications_selects_python_first_and_escapes_from_the_true_time(when):
    p = exit_planner()
    if when == 'preparing':
        to_prefix(p)
        tick(p)
    else:
        to_granted(p)
        tick(p)
    tx = p.exit_handoff
    last = p.last_reference
    stamp_before = p.last_reference_stamp_ns
    for name in ('c1f2_recovery_request_pub', 'c1f1_shadow_enable_publisher', 'c1f2_authority_publisher'):
        getattr(p, name).flags.clear()
        getattr(p, name).messages.clear()
    p.clock.t += 0.011                              # between two publications
    p.external_landing_callback(NS(data=True))
    flags = p.c1f2_recovery_request_pub.flags + p.c1f1_shadow_enable_publisher.flags
    assert flags and all(f == (False, True, False) for f in flags)              # local selection came first
    assert requests(p)[0] == f'{tx.request_id} cancel'
    assert p.c1f1_shadow_enable_publisher.messages[-1].data is False
    assert p.phase == P.RETURN_TO_START and P.RETURN_TO_START not in p.c1f2_cpp_transit_phases()
    assert p.exit_land_sequence == tx.sequence and tx.state == 'landed'
    t_pub = p.clock.t + DT - 0.011
    if when == 'preparing':
        executed = [x[0] for x in tx.prefix.state([t_pub])[:3]]
    else:
        w = h.advance_reference_window(last, elapsed_s=t_pub - stamp_before * 1e-9, sample_dt_s=DT, output_count=1)
        executed = state_of(w)
    ref, python = tick(p, dt=DT - 0.011)
    assert close(state_of(ref), executed, 1e-6)                                 # the published horizon at t_pub
    at_land = h.advance_reference_window(last, elapsed_s=max(0.0, p.clock.t - DT + 0.011 - stamp_before * 1e-9),
                                         sample_dt_s=DT, output_count=1)
    assert not close(state_of(ref), state_of(at_land), 1e-4)                    # not the LAND time's state
    assert not close(state_of(ref), state_of(last), 1e-4)                       # nor an old sample 0 relabelled
    # the old transaction's late results are ignored
    status(p, prepared=True, granted=True, reference_stamp_ns=1)
    assert p.phase == P.RETURN_TO_START and not p.c1f2_cpp_authority_active
    p.external_landing_callback(NS(data=True))                                  # repeated: still the escape
    assert p.phase == P.RETURN_TO_START
    for _ in range(int(25 / DT)):
        tick(p)
        if p.phase == P.LANDING:
            break
    assert p.phase == P.LANDING                                                  # direct return, then LANDING


def test_an_escape_blend_that_fails_its_checks_flies_the_direct_return_as_it_is():
    p = exit_planner()
    to_prefix(p)
    tick(p)
    p.c1f2_bridge_max_jerk = np.full(3, 1e-4)
    p.external_landing_callback(NS(data=True))
    ref, python = tick(p)
    assert p.exit_blend is None and p.phase == P.RETURN_TO_START
    np.testing.assert_array_equal(ref.positions, python.positions)


def test_land_before_the_clearance_completes_is_todays_landing():
    p = exit_planner()
    p.phase_entry_time = T0                          # still climbing
    p.external_landing_callback(NS(data=True))
    assert p.phase == P.LANDING and not p.exit_escape_active


# ---- AT_START -> LANDING ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize('dwell', [0.0, 2.0])
def test_at_start_lands_after_its_dwell_and_holds_when_off(dwell):
    p = exit_planner()
    p.at_start_land_after_s = dwell
    p.return_hover = np.array([START[0], START[1], 2.9])
    p.phase = P.AT_START
    place(p, p.return_hover)
    p.condition_start_time = None
    p.update_state_machine()
    p.clock.t += 1.9
    place(p, p.return_hover)
    p.update_state_machine()
    assert p.phase == P.AT_START
    p.clock.t += 0.2
    place(p, p.return_hover)
    p.update_state_machine()
    assert p.phase == (P.LANDING if dwell else P.AT_START)


# ---- the switch off --------------------------------------------------------------------------------------------------

def test_with_the_switch_off_nothing_new_is_reachable():
    p = exit_planner(switch=False)
    assert not p.exit_preparing() and not p.exit_land()
    for _ in range(12):
        tick(p)
    assert p.c1f2_prepare_active and p.c1f2_prepare_hold_position is not None   # today's world-fixed hold
    assert p.exit_prefix_publisher.messages == []
    assert all(m.header.stamp.sec for m in p.reference_publisher.messages)


HEAD_SOURCE = 'src/tejen_mission/tejen_mission/online_join_planner.py'


def _head_module(tmp_path):
    root = Path(__file__).resolve().parents[3]
    try:
        text = subprocess.run(['git', '-C', str(root), 'show', f'HEAD:{HEAD_SOURCE}'], check=True,
                              capture_output=True, text=True).stdout
    except (OSError, subprocess.CalledProcessError):
        pytest.skip('no git HEAD to compare with')
    if 'c1f2_exit_moving_handoff' in text:
        pytest.skip('HEAD already carries the exit')
    path = tmp_path / 'online_join_planner_head.py'
    path.write_text(text)
    spec = importlib.util.spec_from_file_location('tejen_mission._online_join_planner_head', path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _scenario(planner_cls, module, monkeypatch):
    """return_after_drop 'cpp' as flown on 13 Nov: the clearance, the stationary hold, its recovery transaction, a LAND."""
    ids = (f'{i:032x}' for i in itertools.count(1))
    monkeypatch.setattr(module.uuid, 'uuid4', lambda: NS(hex=next(ids)))
    p = object.__new__(planner_cls)
    base = returning('cpp')
    p.__dict__.update(base.__dict__)
    p.get_logger = base.get_logger
    p._physical_now_s = lambda: p.clock.t
    p._wall_now_s = lambda: p.clock.t
    p.quad_reference_offset_from_tip = lambda: QUAD_FROM_TIP.copy()
    p.tracking_errors = planner_cls.tracking_errors.__get__(p)
    exit_planner(switch=None, cls_planner=p)
    log = []
    for _ in range(40):
        ref, _ = tick(p)
        log.append((p.phase, ref.positions.copy(), ref.velocities.copy(), ref.accelerations.copy()))
    for stamp, prepared in ((10, False), (11, True)):         # the backend: preparing, then a candidate
        status_msg = {'request_id': p.c1f2_reference_recovery.request_id, 'epoch': 2, 'stamp_ns': stamp,
                      'prepared': prepared, 'granted': False, 'reference_stamp_ns': 0, 'reason': ''}
        p.c1f2_recovery_status_callback(NS(data=json.dumps(status_msg)))
        for _ in range(5):
            ref, _ = tick(p)
            log.append((p.phase, ref.positions.copy(), ref.velocities.copy(), ref.accelerations.copy()))
    p.external_landing_callback(NS(data=True))
    ref, _ = tick(p)
    log.append((p.phase, ref.positions.copy(), ref.velocities.copy(), ref.accelerations.copy()))
    messages = {name: [getattr(m, 'data', None) for m in getattr(p, name).messages]
                for name in ('c1f2_recovery_request_pub', 'c1f2_authority_publisher', 'c1f1_shadow_enable_publisher')}
    stamps = [(m.header.stamp.sec, m.header.stamp.nanosec) for m in p.reference_publisher.messages]
    return log, messages, stamps


def test_with_the_switch_off_the_flown_return_is_todays_tick_for_tick(tmp_path, monkeypatch):
    head = _head_module(tmp_path)
    old = _scenario(head.OnlineJoinPlanner, head, monkeypatch)
    new = _scenario(ojp.OnlineJoinPlanner, ojp, monkeypatch)
    (old_log, old_msgs, old_stamps), (new_log, new_msgs, new_stamps) = old, new
    assert [x[0] for x in old_log] == [x[0] for x in new_log]
    assert P.LANDING in [x[0] for x in new_log]
    for a, b in zip(old_log, new_log):
        for x, y in zip(a[1:], b[1:]):
            np.testing.assert_array_equal(x, y)
    sent = new_msgs['c1f2_recovery_request_pub']
    assert old_msgs == new_msgs and [w for w in (' seed ', ' grant ', ' cancel') if any(w in m for m in sent)] == [
        ' seed ', ' grant ', ' cancel']
    assert old_stamps == new_stamps


# ---- the review of 15 Nov: continuity during the C++ return -------------------------------------------------------------

V_CPP = np.array([0.30, -0.10, 0.05])


def cpp_curve(t, base):
    return base + np.outer(np.atleast_1d(t) - T0, V_CPP)


def feed_window(p, stamp_s, receipt_s, base):
    """His window sampled at stamp_s (sample i at stamp_s + i dt, as an exit transaction stamps it), received at receipt_s."""
    times = stamp_s + np.arange(61) * DT
    p.c1f1_shadow_reference = TrajectoryReference(cpp_curve(times, base), np.tile(V_CPP, (61, 1)), np.zeros((61, 3)))
    stamp_ns = int(round(stamp_s * 1e9))
    p.c1f1_shadow_reference_source_stamp_ns, p.c1f1_shadow_reference_source_stamp_s = stamp_ns, stamp_ns * 1e-9
    p.c1f1_shadow_reference_last_receive_time = receipt_s
    p.c1f1_shadow_reference_sample_count = 61


def test_the_cpp_return_advances_each_window_from_its_sample_time_not_its_receipt():
    """Windows of one curve, received 2 ms or 40 ms after their sample time: every publication is the curve at its own
    stamp (the receipt age stepped it by v x the delay difference, 11 mm in the replay)."""
    p = exit_planner()
    to_granted(p)
    base = p.last_reference.positions[0] - V_CPP * (p.clock.t - T0)
    out = []
    for k in range(16):
        tn = p.clock.t + DT
        stamp = tn - 0.045 + 0.001 * (k % 3)                          # his 30 Hz tick, before ours
        feed_window(p, stamp, stamp + (0.002 if k % 2 else 0.040), base)
        tick(p)
        msg = p.reference_publisher.messages[-1]
        t_msg = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        tr = msg.points[0].transforms[0].translation
        out.append((t_msg, np.array([tr.x, tr.y, tr.z])))
    assert p.phase == P.RETURN_TO_START and p.c1f2_cpp_authority_active and p.exit_source_time()
    for t_msg, pos in out[1:]:
        np.testing.assert_allclose(pos, cpp_curve(t_msg, base)[0], atol=1e-6)
    steps = [np.linalg.norm(b[1] - a[1] - V_CPP * (b[0] - a[0])) for a, b in zip(out[1:], out[2:])]
    assert max(steps) < 1e-6


def test_a_stale_window_continues_from_its_sample_time_too():
    """A window that stops arriving: the cached continuation advances it from its sample time (no step at the switch)."""
    p = exit_planner()
    to_granted(p)
    base = p.last_reference.positions[0] - V_CPP * (p.clock.t - T0)
    tn = p.clock.t + DT
    feed_window(p, tn - 0.040, tn - 0.002, base)
    tick(p)
    for _ in range(9):                                                 # nothing new for 0.3 s: past the 0.2 s timeout
        tick(p)
        msg = p.reference_publisher.messages[-1]
        t_msg = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        tr = msg.points[0].transforms[0].translation
        np.testing.assert_allclose([tr.x, tr.y, tr.z], cpp_curve(t_msg, base)[0], atol=1e-6)
    assert p.c1f2_cached_continuation_active


ERR = np.array([0.08, -0.05, 0.03])      # QUAD4 off its reference, inside the start hover's capture (0.12 / 0.15 m)


def tick_err(p, err, dt=DT):
    """tick() with QUAD4 flying the published reference off by err (the recorded error's size)."""
    p.clock.t += dt
    ring_at(p, p.clock.t)
    p.exit_ring_commitment_receive_s = p.clock.t
    pos, vel, acc = published_state(p, p.clock.t)
    place(p, pos + err, vel)
    p.drone_acceleration = acc.copy()
    p.update_state_machine()
    python = None if p.c1f2_cpp_transit_authority_active() else p.build_reference()
    p.update_c1f1_shadow_transfer()
    ref = p.select_c1f2_authority_reference(python)
    p.last_reference, p.last_reference_time_s = ref, p.clock.t
    p.publish_reference(ref)
    return ref, python


def hover_window(p, hover, v=np.zeros(3), t_c=0.0):
    """His window over the start hover (at rest, or drifting through it at v, there at t_c), sampled before our next
    tick (an exit transaction's stamp)."""
    tn = p.clock.t + DT
    stamp_ns = int(round((tn - 0.02) * 1e9))
    times = stamp_ns * 1e-9 + np.arange(61) * DT
    p.c1f1_shadow_reference = TrajectoryReference(hover + np.outer(times - t_c, v), np.tile(v, (61, 1)),
                                                  np.zeros((61, 3)))
    p.c1f1_shadow_reference_source_stamp_ns, p.c1f1_shadow_reference_source_stamp_s = stamp_ns, stamp_ns * 1e-9
    p.c1f1_shadow_reference_last_receive_time = tn - 0.01
    p.c1f1_shadow_reference_sample_count = 61


def arrive(p, err, v=np.zeros(3)):
    """Fly his hover window with QUAD4 off by err until AT_START; return (executed at the switch, the first AT_START
    publication)."""
    hover, t_c = p.return_hover.copy(), p.clock.t + 0.6
    for _ in range(60):
        hover_window(p, hover, v, t_c)
        before = (p.last_reference, p.last_reference_stamp_ns)
        tick_err(p, err)
        if p.phase == P.AT_START:
            t = p.last_reference_stamp_ns * 1e-9
            w = h.advance_reference_window(before[0], elapsed_s=t - before[1] * 1e-9, sample_dt_s=DT, output_count=1)
            return state_of(w), state_of(p.last_reference)
    raise AssertionError('no AT_START')


def test_the_arrival_at_the_start_hover_starts_on_the_cpp_reference_not_the_measured_state():
    p = exit_planner()
    p.at_start_land_after_s = 2.0
    to_granted(p)
    executed, first = arrive(p, ERR)
    assert close(first, executed, 1e-6)               # p, v, a: no step at the switch, with QUAD4 8-10 cm off
    np.testing.assert_allclose(first[0], p.return_hover, atol=1e-6)
    assert p.exit_handoff.state == 'done' and requests(p)[-1].endswith('cancel')
    landing = None
    for k in range(90):                               # the capture and its 2 s dwell stay measured
        tick_err(p, ERR)
        if p.phase == P.LANDING:
            landing = k
            break
    assert landing is not None and 58 <= landing <= 62


def test_the_arrival_with_the_seed_off_steps_by_the_tracking_error():
    """The step the seed removes: today's AT_START transfer restarts from the measured state."""
    p = exit_planner()
    to_granted(p)
    p.exit_seed_transfer = lambda: None
    p.exit_on_transition = lambda new_phase: None
    executed, first = arrive(p, ERR)
    np.testing.assert_allclose(first[0] - executed[0], ERR, atol=1e-6)


@pytest.mark.parametrize('when', ['preparing', 'after the grant'])
def test_the_escape_starts_on_the_executed_reference_with_quad4_off_it(when):
    p = exit_planner()
    if when == 'preparing':
        to_prefix(p)
        tick_err(p, ERR)
    else:
        to_granted(p)
        tick_err(p, ERR)
    t_prev, prev = p.last_reference_stamp_ns * 1e-9, p.last_reference
    p.external_landing_callback(NS(data=True))
    ref, python = tick_err(p, ERR)
    t = p.last_reference_stamp_ns * 1e-9
    w = h.advance_reference_window(prev, elapsed_s=t - t_prev, sample_dt_s=DT, output_count=1)
    assert close(state_of(ref), state_of(w), 1e-6)
    assert close(state_of(python)[:2], state_of(w)[:2], 1e-6)        # the direct return itself starts there (p, v)


@pytest.mark.parametrize('kind', ['escape', 'arrival'])
def test_a_blend_that_fails_its_checks_still_starts_on_the_executed_position_and_velocity(kind):
    """Review 15 Nov, continuity 3: the blend's fallback is the seeded transfer, so a failed blend leaves no p or v step;
    only the acceleration steps, to the transfer's own start (bounded below)."""
    p = exit_planner()
    to_granted(p)
    p.c1f2_bridge_max_jerk = np.full(3, 1e-4)
    if kind == 'escape':
        tick_err(p, ERR)
        t_prev, prev = p.last_reference_stamp_ns * 1e-9, p.last_reference
        p.external_landing_callback(NS(data=True))
        ref, _ = tick_err(p, ERR)
        t = p.last_reference_stamp_ns * 1e-9
        executed = state_of(h.advance_reference_window(prev, elapsed_s=t - t_prev, sample_dt_s=DT, output_count=1))
        first = state_of(ref)
    else:
        executed, first = arrive(p, ERR, v=np.array([0.05, 0.0, 0.0]))
    assert p.exit_blend is None
    assert close(first[:2], executed[:2], 1e-6)
    assert float(np.max(np.abs(first[2] - executed[2]))) < 2.0
