"""multi_drone_control 4 Nov (Wesley: line up with the plate, never get stuck): off the plate's radial line and outside
line_up_radius_m the transit to the ring aims first at the point on that line; within line_up_lateral_m of the line (on
the plate's side) or inside the radius it goes direct, and once direct it stays direct for the phase."""
from types import SimpleNamespace

import numpy as np

from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.online_join_planner import OnlineJoinPlanner, line_up_stage
from tejen_mission.reference_generators import TargetState

RING, PLATE9 = np.array([0.0, 0.0]), np.array([0.0, 0.2293])


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
