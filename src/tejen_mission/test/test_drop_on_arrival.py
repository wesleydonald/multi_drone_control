"""multi_drone_control 4 Nov (RIG-1104-k1m_2/_3/_4): on the 1 m cable the object swung 0.23-0.26 m (median) around the
drop point, so his transit gate (0.18 m and 0.20 m/s held 0.5 s) never let the drop start. drop_on_arrival (default
off) releases at the transit height once the magnet is inside drop_on_arrival_xy_m for 0.1 s: no settle, no descent,
and the climb after it never goes below the transit height. Off keeps his sequence."""
import numpy as np

from tejen_mission.mission_types import MissionPhase as P
from test_m1_contact_attach import planner, tick


def _transit(on, xy_m=0.12):
    p = planner(contact_attach=False, phase=P.TRANSIT_TO_DROP_POINT)
    p.pickup_z_tolerance = 0.20
    p.drop_on_arrival, p.drop_on_arrival_xy_m = on, xy_m
    p.drop_approach_clearance, p.drop_release_clearance, p.drop_clear_height = 0.55, 0.14, 0.17
    p.drop_clear_speed = 0.10
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
