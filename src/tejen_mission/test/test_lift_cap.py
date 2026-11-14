"""multi_drone_control 12 Nov (13 Nov: the object raised on a stand, about ring height): pickup_lift_height counts from
the object, so a raised object put QUAD4's loaded lift that much above the planned transit height. pickup_lift_max_tip_z
(default 0 = off) caps the lift's tip in world z, never less than PICKUP_LIFT_MIN_M above its start."""
import numpy as np
import pytest

from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.online_join_planner import PICKUP_LIFT_MIN_M
from test_m1_contact_attach import QUAD_FROM_TIP, planner

CAP = 1.73          # the rig's planned loaded tip (ring 1.2 + drop_height 0.48 + 0.05)
FLOOR, STAND = 0.03, 1.21      # the object's z: the launch's floor value, and body 6 on a 1.15 m stand
BRANCHES = ('legacy', 'geometry', 'hold')


def _lift(object_z, branch, cap=0.0):
    p = planner(contact_attach=True, phase=P.LIFT_OBJECT)
    p.pickup_attach_clearance, p.pickup_lift_height, p.pickup_lift_speed = 0.03, 1.70, 0.12
    p.latched_pickup_position = np.array([-0.65, 1.61, object_z])
    p.pickup_lift_max_tip_z = cap
    if branch == 'geometry':        # the twin: the geometry-aware contact offset
        p.use_geometry_aware_pickup = True
        p.latched_geometry_pickup_contact_offset = np.array([0.0, 0.0, -0.025])
    elif branch == 'hold':          # the rig: the measured-tip contact hold's body reference
        p.m1_candidate_hold_reference = p.latched_pickup_position + np.array([0.0, 0.0, -0.025]) + QUAD_FROM_TIP
    return p


def _tip_z(p):
    """(start, end) of the lift's tip in world z."""
    prof = p.rate_profile(P.LIFT_OBJECT)
    z = float(p.latched_pickup_position[2])
    return z + float(prof.start_tip_offset[2]), z + float(prof.goal_tip_offset[2])


@pytest.mark.parametrize('branch', BRANCHES)
def test_off_counts_the_lift_from_the_object(branch):
    assert _tip_z(_lift(STAND, branch))[1] - _tip_z(_lift(FLOOR, branch))[1] == pytest.approx(STAND - FLOOR)


@pytest.mark.parametrize('branch', BRANCHES)
def test_on_leaves_the_floor_lift_as_it_was(branch):
    assert _tip_z(_lift(FLOOR, branch, CAP)) == pytest.approx(_tip_z(_lift(FLOOR, branch)))


@pytest.mark.parametrize('branch', BRANCHES)
def test_on_ends_a_raised_objects_lift_at_the_cap(branch):
    start, end = _tip_z(_lift(STAND, branch, CAP))
    assert end == pytest.approx(CAP) and start == pytest.approx(_tip_z(_lift(STAND, branch))[0])


@pytest.mark.parametrize('branch', BRANCHES)
def test_an_object_above_the_cap_still_leaves_its_support(branch):
    start, end = _tip_z(_lift(1.80, branch, CAP))
    assert end - start == pytest.approx(PICKUP_LIFT_MIN_M)
