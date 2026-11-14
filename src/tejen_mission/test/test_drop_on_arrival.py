"""multi_drone_control 4 Nov (RIG-1104-k1m_2/_3/_4): on the 1 m cable the object swung 0.23-0.26 m (median) around the
drop point, so his transit gate (0.18 m and 0.20 m/s held 0.5 s) never let the drop start. drop_on_arrival (default
off) releases at the transit height once the magnet is inside drop_on_arrival_xy_m for 0.1 s: no settle, no descent,
and the climb after it never goes below the transit height. Off keeps his sequence."""
import numpy as np
import pytest
from types import SimpleNamespace

from tejen_mission.online_join_planner import OnlineJoinPlanner

from tejen_mission.mission_types import MissionPhase as P
from test_m1_contact_attach import planner, tick


def _transit(on, xy_m=0.12):
    p = planner(contact_attach=False, phase=P.TRANSIT_TO_DROP_POINT)
    p.pickup_z_tolerance = 0.20
    p.drop_on_arrival, p.drop_on_arrival_xy_m = on, xy_m
    p.drop_approach_clearance, p.drop_release_clearance, p.drop_clear_height = 0.55, 0.14, 0.17
    p.drop_clear_speed = 0.10
    p.drop_descent_speed = 0.10
    return p


def _fly(p, samples, dt=1 / 30):
    """Feed (xy, speed) samples; stop at the first phase change."""
    for xy, speed in samples:
        p.errors = (xy, speed, 0.0)
        tick(p, dt)
        if p.phase != P.TRANSIT_TO_DROP_POINT:
            return


def test_off_keeps_his_gate_and_the_swing_never_passes_it():
    p = _transit(on=False)
    # inside 0.18 m for a third of a second at a time, then swinging out, as flown (k1m_4: longest 0.34 s)
    swing = ([(0.10, 0.10)] * 10 + [(0.26, 0.40)] * 20) * 6
    _fly(p, swing)
    assert p.phase == P.TRANSIT_TO_DROP_POINT


def test_on_drops_at_the_transit_height_on_the_first_pass_inside_the_radius():
    p = _transit(on=True)
    _fly(p, [(0.26, 0.40)] * 20 + [(0.10, 0.40)] * 10)
    assert p.phase == P.DROP_OBJECT
    assert np.allclose(p.desired_tip_offset(), [0.0, 0.0, 0.55])        # held at the transit height, not 0.14


def test_on_ignores_a_single_sample_inside_and_the_radius_is_typed():
    p = _transit(on=True, xy_m=0.08)
    _fly(p, [(0.10, 0.0)] * 30 + ([(0.05, 0.0)] + [(0.20, 0.0)]) * 10)
    assert p.phase == P.TRANSIT_TO_DROP_POINT


def test_the_climb_after_the_drop_never_descends_when_on():
    p = _transit(on=True)
    prof = p.rate_profile(P.CLEAR_DROP_ZONE)
    assert np.isclose(prof.tip_offset_at(0.0)[2], 0.55) and np.isclose(prof.tip_offset_at(60.0)[2], 0.55)
    p.drop_on_arrival = False
    prof = p.rate_profile(P.CLEAR_DROP_ZONE)
    assert np.isclose(prof.tip_offset_at(0.0)[2], 0.14) and np.isclose(prof.tip_offset_at(60.0)[2], 0.17)


# ---- multi_drone_control 11 Nov (Wesley): the operator's FORCE DROP ----

def _press(p):
    from std_msgs.msg import Bool
    p.force_drop_callback(Bool(data=True))


def test_force_drop_skips_the_gates_lowers_and_a_second_press_releases():
    p = _transit(on=False)
    _fly(p, [(0.26, 0.40)] * 10)                      # swinging: his gate does not pass
    assert p.phase == P.TRANSIT_TO_DROP_POINT
    _press(p)
    p.errors = (0.20, 0.40, 0.0)                      # over the ring but too fast for his gate
    tick(p, 1 / 30)
    assert p.phase == P.DESCEND_TO_DROP_HEIGHT       # lowering to the release, the gates skipped
    _press(p)
    tick(p, 1 / 30)
    assert p.phase == P.DROP_OBJECT                  # the second press releases in the lowering


def test_force_drop_over_the_drop_point_and_with_drop_on_arrival():
    p = _transit(on=False)
    p.phase = P.SETTLE_ABOVE_DROP_POINT
    _press(p)
    p.errors = (0.22, 0.40, 0.0)
    tick(p, 1 / 30)
    assert p.phase == P.DESCEND_TO_DROP_HEIGHT
    p = _transit(on=True)
    _press(p)
    p.errors = (0.22, 0.40, 0.0)
    tick(p, 1 / 30)
    assert p.phase == P.DROP_OBJECT                  # drop on arrival has no lowering: released where it is


def test_force_drop_is_ignored_outside_the_drop_phases():
    p = _transit(on=False)
    p.phase = P.LIFT_OBJECT
    _press(p)
    assert not getattr(p, "force_drop_pending", False)
    p.phase = P.TRANSIT_TO_DROP_POINT                # a press before the transit does not carry over
    p.errors = (0.30, 0.40, 0.0)
    tick(p, 1 / 30)
    assert p.phase == P.TRANSIT_TO_DROP_POINT


def test_force_drop_away_from_the_ring_is_dropped_not_kept():
    p = _transit(on=False)
    _press(p)
    p.errors = (0.60, 0.10, 0.0)                      # still 0.6 m out: not over the ring
    tick(p, 1 / 30)
    assert p.phase == P.TRANSIT_TO_DROP_POINT and not p.force_drop_pending
    p.errors = (0.10, 0.40, 0.0)                      # later over the ring: the old press does not fire
    tick(p, 1 / 30)
    assert p.phase == P.TRANSIT_TO_DROP_POINT
    assert any('FORCE DROP ignored' in w for w in p.warnings)


DROP_POINT = np.array([0.3, -0.2, 1.25])
QUAD_OFFSET = np.array([0.0, 0.0, 1.04])


def _commanded(p, offset):
    """A drop point and a last reference that puts the tip at `offset` from it."""
    p.target_state = lambda source: SimpleNamespace(position=DROP_POINT.copy())
    p.quad_reference_offset_from_tip = lambda: QUAD_OFFSET.copy()
    p.last_reference = SimpleNamespace(positions=[DROP_POINT + np.asarray(offset, float) + QUAD_OFFSET])


def test_force_drop_in_the_lowering_holds_the_reference_where_it_was():
    """Te-Jen's review 11 Nov: the release must not step the reference down to the release height (0.25 -> 0.03)."""
    p = _transit(on=False)
    p.phase = P.DESCEND_TO_DROP_HEIGHT
    p.phase_entry_time = p.clock.t
    p.clock.t += 3.0                                   # 0.30 m of the 0.41 m lowering: the tip at 0.25 m
    before = p.desired_tip_offset().copy()
    assert before[2] == pytest.approx(0.25)
    _press(p)
    p.errors = (0.05, 0.10, 0.0)
    tick(p, 0.0)
    assert p.phase == P.DROP_OBJECT
    assert p.desired_tip_offset() == pytest.approx(before)        # no step
    p.phase = P.CLEAR_DROP_ZONE
    prof = p.rate_profile()
    assert prof.start_tip_offset == pytest.approx(before) and prof.goal_tip_offset[2] >= before[2]
    OnlineJoinPlanner.transition_to(p, P.TRANSIT_TO_REATTACH, 'test')
    assert p.drop_hold_offset is None


def test_force_drop_from_the_transit_lowers_from_the_commanded_reference():
    """...and the first press starts the lowering where the transfer reference is, not at the nominal start over the
    drop point (a sideways step of up to the 0.25 m reach)."""
    p = _transit(on=False)
    _commanded(p, [0.18, -0.10, 0.62])
    _press(p)
    p.errors = (0.20, 0.40, 0.0)
    tick(p, 1 / 30)
    assert p.phase == P.DESCEND_TO_DROP_HEIGHT
    assert p.rate_profile().start_tip_offset == pytest.approx([0.18, -0.10, 0.62])
    assert p.rate_profile().goal_tip_offset == pytest.approx([0.0, 0.0, 0.14])


def test_a_press_left_pending_dies_with_its_phase():
    p = _transit(on=False)
    _press(p)
    assert p.force_drop_pending
    OnlineJoinPlanner.transition_to(p, P.TAKEN_BACK, 'take-back')
    assert not p.force_drop_pending


def test_a_press_left_pending_dies_with_a_disarm():
    """Te-Jen's review 11 Nov: DISARM leaves the phase as it is, so it clears the press itself."""
    from std_msgs.msg import String
    p = _transit(on=False)
    _press(p)
    assert p.force_drop_pending
    p.drone_command_callback(String(data='DISARM'))
    assert not p.force_drop_pending
