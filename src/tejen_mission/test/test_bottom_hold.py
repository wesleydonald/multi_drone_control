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
