"""Takeoff from the mat on the rig (14 Oct): WAIT_FOR_TAKEOFF holds the ground pose (his MPC can take off on an earlier
copy of TAKEOFF than this planner, t6/t6f: it drifted 0.4-0.5 m on a reference that followed it), and the timed TAKEOFF
climb starts only once the vehicle is off the mat (his MPC got TAKEOFF 18 s after this planner in t6h_4)."""
import numpy as np
import pytest

from tejen_mission.mission_types import MissionPhase as P
from test_floor_join_delivery_rejoin import MAT, command, floor_planner, place, tick


def _rig(p):
    p.min_reference_z = 0.58                  # the rig's body-reference floor (m1_ground_ring.yaml)
    return p


def test_wait_for_takeoff_holds_the_ground_pose_while_it_climbs_early():
    p = _rig(floor_planner('floor_join'))
    p.build_reference()                                           # on the mat: the hold is the mat
    place(p, MAT + np.array([0.20, 0.30, 0.25]))                 # his MPC already lifting and drifting
    ref = p.build_reference()
    assert ref.positions[0][:2] == pytest.approx(MAT[:2])        # held over the mat, not following the drift
    assert ref.positions[0][2] == pytest.approx(0.58)            # raised to the floor of references


def test_the_hold_follows_the_vehicle_while_it_is_on_the_ground():
    p = _rig(floor_planner('floor_join'))
    p.build_reference()
    place(p, MAT + np.array([0.40, 0.0, 0.01]))                  # carried to another spot on the floor
    assert p.build_reference().positions[0][:2] == pytest.approx((MAT + [0.40, 0.0, 0.0])[:2])


def test_the_takeoff_start_is_the_ground_hold_when_it_lifted_first():
    p = _rig(floor_planner('floor_join'))
    p.build_reference()
    place(p, MAT + np.array([0.20, 0.30, 0.40]))
    command(p, 'TAKEOFF')
    assert p.phase == P.TAKEOFF
    assert p.takeoff_start_position == pytest.approx(MAT)


def test_the_climb_waits_until_it_is_off_the_mat():
    p = _rig(floor_planner('floor_join'))
    command(p, 'TAKEOFF')
    assert p.phase == P.TAKEOFF and p.takeoff_airborne is False
    for _ in range(50):                                           # 5 s on the mat (his MPC not flying yet)
        tick(p, 0.1)
    assert p.phase_elapsed() == pytest.approx(0.0)
    place(p, MAT + np.array([0.0, 0.0, 0.05]))                   # it lifts
    tick(p, 0.1)
    assert p.takeoff_airborne is True
    tick(p, 0.5)
    assert p.phase_elapsed() == pytest.approx(0.6)               # timed from its last tick on the mat


def test_without_a_reference_floor_the_climb_runs_on_time_as_before():
    p = floor_planner('floor_join')                               # min_reference_z 0: the profile must lift it
    command(p, 'TAKEOFF')
    tick(p, 0.1)
    assert p.takeoff_airborne is True
