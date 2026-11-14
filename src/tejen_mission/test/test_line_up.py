"""multi_drone_control 4 Nov (Wesley: line up with the plate, never get stuck): off the plate's radial line and outside
line_up_radius_m the transit to the ring aims first at the point on that line; within line_up_lateral_m of the line (on
the plate's side) or inside the radius it goes direct, and once direct it stays direct for the phase."""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.online_join_planner import OnlineJoinPlanner, line_up_gap_point, line_up_stage
from tejen_mission.reference_generators import TargetState

RING, PLATE9 = np.array([0.0, 0.0]), np.array([0.0, 0.2293])
LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'partner_mission.launch.py'


def test_stage_on_the_plate_line_unless_lined_up_or_inside():
    assert np.allclose(line_up_stage(RING, PLATE9, np.array([1.6, 2.0]), 1.2, 0.25), [0.0, 1.2])
    assert line_up_stage(RING, PLATE9, np.array([0.2, 1.8]), 1.2, 0.25) is None        # 0.2 m off the line: in
    assert line_up_stage(RING, PLATE9, np.array([0.5, 0.5]), 1.2, 0.25) is None        # inside the radius
    assert np.allclose(line_up_stage(RING, PLATE9, np.array([0.0, -1.6]), 1.2, 0.25), [0.0, 1.2])  # far side


def _planner(quad, on=True):
    log = SimpleNamespace(info=lambda *_: None)
    return SimpleNamespace(line_up=on, line_up_radius_m=1.2, line_up_lateral_m=0.25, phase=P.TRANSIT_TO_DROP_POINT,
                           payload_position=np.array([0.0, 0.0, 1.0]), attachment_position=np.array([0.0, 0.2293, 1.02]),
                           drone_position=np.array([*quad, 2.5]), get_logger=lambda: log)


def _target(p):
    body = TargetState(np.array([0.0, 0.0, 2.56]), np.zeros(3), np.zeros(3))
    return OnlineJoinPlanner.line_up_target(p, body).position


def test_staging_keeps_the_height_and_latches_direct_once_lined_up():
    p = _planner([1.6, 2.0])
    assert np.allclose(_target(p), [0.0, 1.2, 2.56])
    p.drone_position = np.array([0.1, 1.3, 2.5])            # reached the line
    assert np.allclose(_target(p), [0.0, 0.0, 2.56])
    p.drone_position = np.array([0.4, 1.4, 2.5])            # drifts off the band again: stays direct
    assert np.allclose(_target(p), [0.0, 0.0, 2.56])


def test_off_or_without_poses_is_his_target():
    assert np.allclose(_target(_planner([1.6, 2.0], on=False)), [0.0, 0.0, 2.56])
    p = _planner([1.6, 2.0]); p.attachment_position = None
    assert np.allclose(_target(p), [0.0, 0.0, 2.56])


# ── 12 Nov (the 13 Nov trials, starts at 9, 1 and 5 o'clock): line_up:=gap ─────────────────────────────────────────

def _clock(hour, r):
    a = math.radians(-30.0 * hour)              # our plates: clockwise from +x seen from above
    return np.array([r * math.cos(a), r * math.sin(a)])


def test_the_gap_offsets_from_the_plate_and_the_carriers():
    from importlib import util
    spec = util.spec_from_file_location('pm', str(LAUNCH))
    pm = util.module_from_spec(spec); spec.loader.exec_module(pm)
    assert pm.line_up_gap_offsets('9', [90.0, 330.0, 210.0]) == [0.0, 120.0, 240.0]
    assert pm.line_up_gap_offsets(3, [30.0, 270.0, 150.0]) == [0.0, 120.0, 240.0]


def test_the_nearest_gap_line_for_each_start():
    for hour in (9, 1, 5):
        p = line_up_gap_point(RING, PLATE9, _clock(hour, 1.4), [0.0, 120.0, 240.0])
        assert np.allclose(p / np.linalg.norm(p), _clock(hour, 1.0), atol=1e-9), hour
    assert np.allclose(line_up_gap_point(RING, PLATE9, _clock(1, 1.4), [0.0]), PLATE9 / 0.2293)   # [0]: the plate


def _gap_planner(quad, phase=P.TRANSIT_TO_DROP_POINT):
    p = _planner(quad)
    p.line_up_gap_offsets_deg = [0.0, 120.0, 240.0]
    p.phase = phase
    return p


def test_gap_mode_flies_straight_in_from_1_and_5_and_stages_from_behind_a_carrier():
    for hour in (9, 1, 5):
        p = _gap_planner(_clock(hour, 1.4))
        body = TargetState(np.array([0.0, 0.0, 2.56]), np.zeros(3), np.zeros(3))
        assert np.allclose(OnlineJoinPlanner.line_up_target(p, body, drop=True).position, [0.0, 0.0, 2.56]), hour
    p = _gap_planner(_clock(3, 1.4))            # behind the carrier on 3: stages on the nearer gap's line at 1.2 m
    body = TargetState(np.array([0.0, 0.0, 2.56]), np.zeros(3), np.zeros(3))
    st = OnlineJoinPlanner.line_up_target(p, body, drop=True).position
    assert np.isclose(np.linalg.norm(st[:2]), 1.2) and np.isclose(st[2], 2.56)
    assert min(np.linalg.norm(st[:2] / 1.2 - _clock(h, 1.0)) for h in (1, 5)) < 1e-9


def test_plate_mode_and_the_reattach_transit_keep_the_plate_line():
    body = TargetState(np.array([0.0, 0.0, 2.56]), np.zeros(3), np.zeros(3))
    p = _planner(_clock(1, 1.4))                # line_up:=true: today's staging on plate 9's line
    assert np.allclose(OnlineJoinPlanner.line_up_target(p, body, drop=True).position, [0.0, 1.2, 2.56])
    p = _gap_planner(_clock(1, 1.4), phase=P.TRANSIT_TO_REATTACH)   # the attach transit: the plate's line
    assert np.allclose(OnlineJoinPlanner.line_up_target(p, body, drop=False).position, [0.0, 1.2, 2.56])


def test_the_chosen_gap_is_kept_for_the_phase_while_the_ring_moves():
    """Te-Jen 12 Nov: once chosen, the gap stays for the approach (ring movement cannot switch it); it moves and turns
    with the ring; a new phase chooses again."""
    p = _gap_planner(_clock(3, 1.4) + np.array([0.15, 0.0]))     # behind the carrier on 3, a little toward 1 o'clock
    body = TargetState(np.array([0.0, 0.0, 2.56]), np.zeros(3), np.zeros(3))
    first = OnlineJoinPlanner.line_up_target(p, body, drop=True).position[:2]
    assert np.allclose(first / 1.2, _clock(1, 1.0), atol=1e-9)
    p.drone_position = np.array([*(_clock(3, 1.4) - np.array([0.15, 0.0])), 2.5])   # now nearer 5 o'clock's gap
    p.payload_position = np.array([0.3, 0.0, 1.0]); p.attachment_position = np.array([0.3, 0.2293, 1.02])
    again = OnlineJoinPlanner.line_up_target(p, body, drop=True).position[:2]
    assert np.allclose((again - [0.3, 0.0]) / 1.2, _clock(1, 1.0), atol=1e-9)        # still plate 1's gap, moved
    p.phase = P.SETTLE_ABOVE_DROP_POINT
    p.payload_position = np.array([0.0, 0.0, 1.0]); p.attachment_position = np.array([0.0, 0.2293, 1.02])
    third = OnlineJoinPlanner.line_up_target(p, body, drop=True).position[:2]
    assert np.allclose(third / 1.2, _clock(5, 1.0), atol=1e-9)                         # a new phase chooses again
