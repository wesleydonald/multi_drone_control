"""contact_abort_xy_debounce_s (multi_drone_control 29 Oct; DECISIONS_1355 section 2): the contact descent's 8 cm
position restart waits for a continuous violation on fresh tip samples (RIG-1029-sq5_2: 6 of 6 descents left on one tick
as the magnet swung on its 1 m cable); 0 keeps his single-tick restart, and the speed restart stays immediate."""
import numpy as np
import pytest

from tejen_mission.mission_types import MissionPhase as P
from test_m1_contact_attach import descend, planner, tick


def _rig(debounce):
    p = planner(phase=P.MATCH_VELOCITY)
    descend(p)
    p.contact_abort_xy_threshold, p.contact_abort_xy_velocity_threshold = 0.08, 0.25
    p.contact_abort_xy_debounce_s = debounce
    p.contact_bottom_hold = False
    return p


def _move(p):
    """A new tip sample (Motive repeats the last one while it has lost the marker)."""
    p.measured_magnet_tip_position = p.measured_magnet_tip_position + np.array([1e-4, 0.0, 0.0])


def _out(p, seconds, dt=1 / 30, xy=0.09, speed=0.05):
    for _ in range(round(seconds / dt)):
        p.errors = (xy, speed, 0.0)
        _move(p)
        tick(p, dt)
        if p.phase != P.DESCEND_TO_ATTACHMENT:
            return


def test_off_keeps_his_single_tick_restart():
    p = _rig(0.0)
    _out(p, 1 / 30)
    assert p.phase == P.APPROACH_ABOVE_TARGET


def test_an_excursion_shorter_than_the_debounce_keeps_descending():
    p = _rig(0.3)
    _out(p, 0.2)
    assert p.phase == P.DESCEND_TO_ATTACHMENT


def test_a_continuous_violation_restarts_after_the_debounce():
    p = _rig(0.3)
    _out(p, 0.25)
    assert p.phase == P.DESCEND_TO_ATTACHMENT
    _out(p, 0.1)
    assert p.phase == P.APPROACH_ABOVE_TARGET


def test_recovery_inside_the_bound_resets_the_count():
    p = _rig(0.3)
    for _ in range(4):                                   # 0.2 s out, one tick back in, four times: 0.8 s out in all
        _out(p, 0.2)
        p.errors = (0.05, 0.05, 0.0)
        tick(p, 1 / 30)
    assert p.phase == P.DESCEND_TO_ATTACHMENT


def test_the_speed_restart_stays_immediate():
    p = _rig(0.3)
    _out(p, 1 / 30, xy=0.03, speed=0.30)
    assert p.phase == P.APPROACH_ABOVE_TARGET


def test_missing_tip_samples_earn_no_time():
    p = _rig(0.3)
    _out(p, 0.1)
    for _ in range(9):                                   # 0.3 s with no new tip sample (inside the 0.5 s timeout)
        p.clock.t += 1 / 30
        p.last_drone_time = p.last_attachment_pose_time = p.clock.t
        p.errors = (0.09, 0.05, 0.0)
        p.update_state_machine()
    assert p.phase == P.DESCEND_TO_ATTACHMENT
    _out(p, 0.2)                                         # fresh again: the count restarted at the gap
    assert p.phase == P.DESCEND_TO_ATTACHMENT


def test_a_stale_tip_restarts_the_count():
    p = _rig(0.3)
    _out(p, 0.2)
    p.last_magnet_tip_pose_time = p.clock.t - 1.0       # older than input_timeout_s
    assert not p.contact_position_lost(0.09) and p.contact_abort_xy_since is None


def test_a_new_attempt_starts_its_own_count():
    p = _rig(0.3)
    _out(p, 0.2)
    p.transition_to(P.APPROACH_ABOVE_TARGET, 'test: an operator restart')
    p.transition_to(P.DESCEND_TO_ATTACHMENT, 'test: down again')
    _out(p, 0.2)
    assert p.phase == P.DESCEND_TO_ATTACHMENT


def test_the_bottom_continues_the_descents_count():
    p = _rig(0.3)
    _out(p, 0.2)
    p.transition_to(P.ATTACH_READY, 'test: the profile completed while out')
    for _ in range(5):                                   # a fresh count at the bottom would need 9 more ticks
        p.errors = (0.09, 0.05, 0.0)
        _move(p)
        tick(p, 1 / 30)
        if p.phase != P.ATTACH_READY:
            break
    assert p.phase == P.APPROACH_ABOVE_TARGET


@pytest.mark.parametrize('debounce', [0.0, 0.3])
def test_the_debounce_never_reports_a_contact(debounce):
    p = _rig(debounce)
    _out(p, 0.2)
    assert p.phase in (P.DESCEND_TO_ATTACHMENT, P.APPROACH_ABOVE_TARGET) and not p.contact_reported


def test_a_frozen_tip_marker_earns_no_time_and_restarts_the_count():
    """RIG-1029-sq5_2: the tip's pose repeated for ~0.2 s at a time with fresh timestamps."""
    p = _rig(0.3)
    _out(p, 0.2)
    for _ in range(9):                                   # 0.3 s of the same pose, stamped fresh every tick
        p.errors = (0.09, 0.05, 0.0)
        tick(p, 1 / 30)
    assert p.phase == P.DESCEND_TO_ATTACHMENT
    _out(p, 0.2)                                         # moving again: counted from the first new sample
    assert p.phase == P.DESCEND_TO_ATTACHMENT
