"""multi_drone_control 11 Nov (RIG-1111-fm9c_6): the bigger object came off in the lift and slid to y 2.16 m; his retry
flew to it and QUAD4 left the tracked volume at (0.29, 2.34, 2.89). pickup_box_m (0 = off, typed on the rig): an object
further out than that in x or y is not gone after; he gives up and the fleet takes the vehicle back."""
import numpy as np
from types import SimpleNamespace

from tejen_mission.mission_types import MissionPhase as P
from test_m1_contact_attach import planner


def _p(phase, xy, box=1.5, stage='idle', contact_attach=True):
    p = planner(contact_attach=contact_attach, phase=phase)
    p.pickup_box_m, p.pickup_box_since = box, None
    p.pickup_object_position = np.array([xy[0], xy[1], 0.04])
    p.m1_pickup_release = SimpleNamespace(stage=stage)
    p.gave_up = []
    p.contact_give_up = p.gave_up.append
    return p


def _steps(p, seconds, dt=0.05):
    out = False
    for _ in range(round(seconds / dt)):
        p.clock.t += dt
        out = p.pickup_box_step() or out
    return out


def test_an_object_outside_the_box_is_given_up_after_the_dwell():
    p = _p(P.APPROACH_ABOVE_PICKUP, (-0.07, 2.16))
    assert not _steps(p, 0.2) and not p.gave_up
    assert _steps(p, 0.3) and len(p.gave_up) == 1 and 'y +2.16' in p.gave_up[0] and '1.50 m pickup box' in p.gave_up[0]


def test_inside_the_box_off_or_not_going_for_it_nothing_happens():
    for kw in (dict(phase=P.APPROACH_ABOVE_PICKUP, xy=(-0.6, 1.25)),             # inside
               dict(phase=P.APPROACH_ABOVE_PICKUP, xy=(-0.07, 2.16), box=0.0),   # off
               dict(phase=P.LIFT_OBJECT, xy=(-0.5, 1.6)),                        # carried past the box: not a pickup
               dict(phase=P.TRANSIT_TO_DROP_POINT, xy=(0.0, 1.9)),
               dict(phase=P.APPROACH_ABOVE_PICKUP, xy=(-0.07, 2.16), contact_attach=False)):
        p = _p(**kw)
        assert not _steps(p, 1.0) and not p.gave_up, kw


def test_a_retry_after_the_object_fell_counts_whatever_the_phase():
    p = _p(P.LIFT_OBJECT, (-0.07, 2.16), stage='clearing')
    assert _steps(p, 0.5) and len(p.gave_up) == 1


def test_one_bad_pose_does_not_trip_it():
    p = _p(P.DESCEND_TO_PICKUP, (0.2, 2.4))
    _steps(p, 0.2)
    p.pickup_object_position = np.array([0.2, 1.2, 0.04])
    assert not _steps(p, 0.5) and not p.gave_up
