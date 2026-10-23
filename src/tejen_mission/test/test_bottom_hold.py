"""The bottom hold and the lift-done signal (multi_drone_control plan 2026-10-14 WP3, contact_bottom_hold): ATTACH_READY
waits at the bottom with the magnet ON for the contact instead of leaving on the first 8 cm / 0.25 m/s excursion (on
14 Oct the tip spent under 1 s at the bottom of every no-catch descent); it leaves on a gross error or on his quiet-miss
rule; and /join_planner/proof_lift_complete tells our co-motion proof when his lift is done."""
import numpy as np
import pytest

from tejen_mission.m2_irl_attachment_policy import QuietMissTracker
from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.online_join_planner import RateProfile
from test_m1_contact_attach import Recorder, planner, tick


def _hold(**kw):
    p = planner(phase=P.ATTACH_READY)
    p.contact_bottom_hold = True
    p.contact_gross_speed, p.contact_gross_xy, p.contact_face_below_m = 0.35, 0.12, 0.020
    p.contact_quiet_min_xy, p.contact_max_misses = 0.035, 3
    p.contact_quiet = QuietMissTracker(wait_s=4.0, improvement_m=0.005)
    p.contact_misses, p.contact_bottom_entries = 0, 1
    p.contact_gross_t0 = p.contact_below_t0 = None
    p.contact_first_descend_s = p.clock.t
    p.contact_give_up_s = 45.0
    p.transfer_generator.reset = lambda: None
    # the measured tip, still, where p.errors puts it (a quiet miss needs a fresh, slow tip: Tejen 15 Oct)
    p.measured_contact_tip_offset = lambda: (np.array([p.errors[0], 0.0, p.errors[2] + p.final_attach_offset_z])
                                             if getattr(p, 'errors', None) else None)
    p.last_magnet_tip_pose_time = p.clock.t
    p.contact_quiet_speed, p.ring_quaternion, p.last_ring_pose_time = 0.03, None, None
    p.contact_tip_last, p.contact_tip_vel = None, np.zeros(3)
    for k, v in kw.items():
        setattr(p, k, v)
    return p


def test_a_swinging_tip_that_used_to_abort_now_waits_at_the_bottom():
    p = _hold()
    for k in range(30):                                   # 3 s of a 4-8 cm swing at 0.2-0.3 m/s
        p.errors = (0.04 + 0.04 * abs(np.sin(k)), 0.20 + 0.10 * abs(np.cos(k)), 0.0)
        tick(p, 0.1)
    assert p.phase == P.ATTACH_READY                      # the old guard left at 8 cm / 0.25 m/s at once


def test_a_contact_at_the_bottom_starts_the_lift_proof():
    p = _hold()
    p.errors = (0.03, 0.05, 0.0)
    tick(p, 0.5)
    p.contact_reported = True
    tick(p, 0.1)
    assert p.phase == P.LIFT_PROOF


def test_a_quiet_miss_lifts_clear_to_retry_then_gives_up_after_three():
    p = _hold()
    p.errors = (0.05, 0.02, 0.0)                          # resting 5 cm off the plate, no progress
    for _ in range(45):
        tick(p, 0.1)
        if p.phase != P.ATTACH_READY:
            break
    assert p.phase == P.APPROACH_ABOVE_TARGET and p.contact_misses == 1
    assert any('quiet miss 1/3' in m for m in p.infos)
    for n in (2, 3):
        p.phase = P.MATCH_VELOCITY
        p.transition_to(P.ATTACH_READY, 'test: back at the bottom')
        for _ in range(45):
            tick(p, 0.1)
            if p.phase != P.ATTACH_READY:
                break
    assert p.contact_misses == 3 and p.phase == P.TAKEN_BACK


def test_progress_toward_the_plate_resets_the_quiet_timer():
    p = _hold()
    xy = 0.09
    for _ in range(80):                                   # 8 s, closing 6 mm a second: never quiet
        p.errors = (xy, 0.02, 0.0)
        xy = max(0.036, xy - 0.006)
        tick(p, 0.1)
        if xy == 0.036:
            break
    assert p.phase == P.ATTACH_READY and p.contact_misses == 0


@pytest.mark.parametrize('errors,why', [((0.20, 0.05, 0.0), 'moving or far off'),
                                        ((0.03, 0.50, 0.0), 'moving or far off'),
                                        ((0.03, 0.05, -0.05), 'sunk below the plate')])
def test_gross_errors_leave_at_once(errors, why):
    p = _hold()
    p.errors = errors
    for _ in range(5):
        tick(p, 0.1)
    assert p.phase == P.APPROACH_ABOVE_TARGET and any(why in m for m in p.infos)


def test_without_the_switch_the_old_guard_stays():
    p = planner(phase=P.ATTACH_READY)
    p.errors = (0.06, 0.05, 0.0)
    tick(p, 0.1)
    assert p.phase == P.APPROACH_ABOVE_TARGET


def test_lift_complete_is_published_when_his_proof_lift_reaches_its_goal():
    p = _hold()
    p.proof_lift_publisher = Recorder()
    p.contact_lift_published = p.contact_lift_published_t = None
    p.phase = P.LIFT_PROOF
    p.phase_entry_time = p.clock.t
    p.contact_proof_profile = RateProfile(np.zeros(3), np.array([0.0, 0.0, 0.03]), 0.02)   # 1.5 s
    p.contact_reported = True
    tick(p, 0.1)
    assert [m.data for m in p.proof_lift_publisher.messages] == [False]
    tick(p, 1.5)
    assert p.proof_lift_publisher.messages[-1].data is True
    tick(p, 0.1)
    assert len(p.proof_lift_publisher.messages) == 2      # on a change, then twice a second


def test_lift_complete_without_a_measured_profile_uses_its_nominal_time():
    p = _hold()
    p.phase, p.phase_entry_time, p.contact_proof_profile = P.LIFT_PROOF, p.clock.t, None
    p.proof_rise_max, p.proof_rise_rate = 0.03, 0.02
    p.clock.t += 1.7
    assert not p.contact_proof_lift_complete()
    p.clock.t += 0.2
    assert p.contact_proof_lift_complete()


def test_the_hold_judges_the_tip_against_the_plate_not_the_offset_target():
    """contact_dx/dy aim the body: his tip target then sits that far off the plate. A tip parked on that target 5 cm
    off the plate read 0 and was held for the whole give-up time (critic 15 Oct, F3); now it is a quiet miss."""
    p = _hold()
    p.errors = (0.0, 0.05, 0.0)                           # on his (offset) tip target
    p.measured_contact_tip_offset = lambda: np.array([0.05, 0.0, 0.01])
    for _ in range(45):
        tick(p, 0.1)
        if p.phase != P.ATTACH_READY:
            break
    assert p.phase == P.APPROACH_ABOVE_TARGET and p.contact_misses == 1
    q = _hold()
    q.errors = (0.05, 0.05, 0.0)                          # 5 cm off his target but on the plate
    q.measured_contact_tip_offset = lambda: np.array([0.01, 0.0, 0.01])
    for _ in range(60):
        tick(q, 0.1)
    assert q.phase == P.ATTACH_READY and q.contact_misses == 0


def test_a_swinging_tip_is_not_a_quiet_miss_and_stale_data_restarts_the_timer():
    """Tejen 15 Oct: a quiet miss needs the tip slow (0.03 m/s, his M2 value) on fresh data."""
    p = _hold()
    p.errors = (0.05, 0.05, 0.0)
    off = {'x': 0.05}
    p.measured_contact_tip_offset = lambda: np.array([off['x'], 0.0, 0.01])
    for k in range(80):                                    # 8 s swinging 2 cm at 0.65 Hz around 5 cm off
        off['x'] = 0.05 + 0.02 * np.sin(2 * np.pi * 0.65 * k * 0.1)
        p.last_magnet_tip_pose_time = p.clock.t
        tick(p, 0.1)
    assert p.phase == P.ATTACH_READY and p.contact_misses == 0
    q = _hold()
    q.errors = (0.05, 0.05, 0.0)
    for _ in range(30):                                    # 3 s still: quiet time earned
        tick(q, 0.1)
    q.measured_contact_tip_offset = lambda: None           # stale: the timer starts again
    tick(q, 0.1)
    q.measured_contact_tip_offset = lambda: np.array([0.05, 0.0, 0.01])
    for _ in range(30):                                    # 3 s more: not 4 s since the stale sample
        tick(q, 0.1)
    assert q.phase == P.ATTACH_READY and q.contact_misses == 0


def test_the_face_below_the_plate_is_measured_along_the_plate_normal():
    """Tejen 15 Oct: on a ring tilted 10 deg about x, a tip 6 cm off the plate point along the ring's +y reads 1 cm
    below it in world z but sits on the plate's own plane: no exit; 25 mm below the plane is one."""
    p = _hold()
    a = np.radians(10.0)
    p.ring_pose_topic = '/payload/motion_capture_state'
    p.ring_quaternion = (np.cos(a / 2), np.sin(a / 2), 0.0, 0.0)
    p.last_ring_pose_time = p.clock.t
    p.errors = (0.06, 0.05, 0.0)
    rot = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    on_plane = rot @ np.array([0.0, 0.06, 0.0])
    p.measured_contact_tip_offset = lambda: on_plane
    for _ in range(5):
        p.last_ring_pose_time = p.clock.t
        tick(p, 0.1)
    assert p.phase == P.ATTACH_READY
    below = rot @ np.array([0.0, 0.03, -0.025])
    p.measured_contact_tip_offset = lambda: below
    for _ in range(5):
        p.last_ring_pose_time = p.clock.t
        tick(p, 0.1)
    assert p.phase == P.APPROACH_ABOVE_TARGET and any('below the plate' in m for m in p.infos)


def test_input_loss_restarts_the_quiet_timer_on_the_real_return_path():
    """Tejen 15 Oct, round 2: his timer callback returns early on input loss; our quiet-miss time and tip speed must
    not carry across it (QuietMissTracker credits the gap when data resumes)."""
    p = _hold()
    p.errors = (0.05, 0.05, 0.0)
    for _ in range(30):                                   # 3 s still, off the plate: quiet time earned
        tick(p, 0.1)
    assert p.contact_quiet.elapsed_s > 2.5
    p._last_planner_callback_start = None
    p.planner_callback_budget_ms, p.planner_deadline_miss_count = 1e9, 0
    p.publish_committed_trajectory = p.m2c_ground_reference_enabled = p.pickup_proof_enabled = False
    p.inputs_ready = lambda: False
    p.invalidate_pickup_alignment = p.invalidate_target_alignment = lambda: None
    p.m2b_quiet_miss = None
    p.m1_takeoff_mpc_mode_publisher = None                # his gentle-takeoff MPC mode (off)
    p.timer_callback()                                    # the input-loss return
    assert p.contact_quiet.elapsed_s == 0.0 and p.contact_tip_last is None
    for _ in range(30):                                   # 3 s more: not 4 s since the loss
        tick(p, 0.1)
    assert p.phase == P.ATTACH_READY and p.contact_misses == 0


def test_a_stale_ring_pose_pauses_the_plate_frame_checks():
    """Tejen 15 Oct, round 2: with ring_pose_topic configured but stale, no world-axes fallback: the below-the-plate
    exit and the quiet miss pause; the gross exits still run on his tracking errors."""
    p = _hold()
    p.ring_pose_topic = '/payload/motion_capture_state'
    p.ring_quaternion, p.last_ring_pose_time = (1.0, 0.0, 0.0, 0.0), p.clock.t - 5.0     # stale
    p.errors = (0.05, 0.05, -0.05)
    p.measured_contact_tip_offset = lambda: np.array([0.05, 0.0, -0.05])                  # 5 cm "below" in world z
    for _ in range(50):
        tick(p, 0.1)
    assert p.plate_frame_tip_offset() is None
    assert p.phase == P.ATTACH_READY and p.contact_misses == 0
    p.errors = (0.20, 0.05, 0.0)                          # far off: the gross exit still acts
    for _ in range(5):
        tick(p, 0.1)
    assert p.phase == P.APPROACH_ABOVE_TARGET
