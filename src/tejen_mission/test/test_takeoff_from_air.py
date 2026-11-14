"""multi_drone_control 4 Nov: a TAKEOFF that starts more than TAKEOFF_FROM_AIR_M above the boot ground (released in
the air by our planner) climbs from where the vehicle is, at least TAKEOFF_FROM_AIR_CLIMB_M; a floor start keeps his
takeoff_height (RIG-1104-nr9m_2: his old latch left a tired QUAD4 25 cm short for 38 s)."""
from types import SimpleNamespace

import numpy as np

from tejen_mission.online_join_planner import (OnlineJoinPlanner, TAKEOFF_FROM_AIR_CLIMB_M)

ROD = 1.04


def _planner(start_z, from_air, plate_z=1.0, approach=0.30):
    log = SimpleNamespace(info=lambda *_: None)
    return SimpleNamespace(
        nominal_quad_from_tip_offset=lambda: np.array([0.0, 0.0, ROD]),
        is_delivery_rejoin=False, is_floor_join=True, attach_cpp_transit_active=False,
        approach_offset_z=approach, attachment_position=np.array([0.0, 0.23, plate_z]),
        takeoff_height=0.20, takeoff_start_position=np.array([0.0, 1.2, start_z]), takeoff_from_air=from_air,
        get_logger=lambda: log, mission=SimpleNamespace(name='floor_join', takeoff_exit=SimpleNamespace(value='x')))


def test_air_start_at_the_approach_height_latches_on_the_least_climb():
    p = _planner(start_z=1.0 + 0.30 + ROD, from_air=True)
    OnlineJoinPlanner.latch_takeoff_climb(p)
    assert p.takeoff_climb_height == TAKEOFF_FROM_AIR_CLIMB_M


def test_air_start_below_the_approach_height_climbs_to_it():
    p = _planner(start_z=2.10, from_air=True)
    OnlineJoinPlanner.latch_takeoff_climb(p)
    assert np.isclose(p.takeoff_climb_height, 1.0 + 0.30 + ROD - 2.10)


def test_floor_start_keeps_his_least_climb():
    p = _planner(start_z=2.30, from_air=False)
    OnlineJoinPlanner.latch_takeoff_climb(p)
    assert p.takeoff_climb_height == 0.20
