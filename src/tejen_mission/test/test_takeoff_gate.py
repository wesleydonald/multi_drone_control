"""multi_drone_control 5 Nov (RIG-1105-nr1r): with takeoff_gate_topic set (our combined rig launch, a start-welded QUAD4)
a TAKEOFF before our release is held and goes at the release, latched where QUAD4 is then; while held his MPC (which
got the same TAKEOFF) is given a hover where QUAD4 is, not the boot mat. Without the topic nothing changes."""
from types import SimpleNamespace

import numpy as np
import pytest

from tejen_mission.mission_types import MissionPhase as P
from tejen_mission.online_join_planner import OnlineJoinPlanner
from test_floor_join_delivery_rejoin import MAT, command, floor_planner, place

RELEASE = SimpleNamespace(data='{"running": true}')


def _gated():
    p = floor_planner('floor_join')
    p.takeoff_gate_topic, p.takeoff_gate_open, p.takeoff_held = '/partner/release', False, False
    p.build_reference()                                        # booted welded on its plate on the floor
    return p


def test_a_takeoff_before_the_release_is_held_then_goes_from_where_quad4_is():
    p = _gated()
    place(p, MAT + np.array([0.0, 0.0, 1.30]))               # lifted with the ring, our tracker flying it
    command(p, 'TAKEOFF')
    assert p.phase == P.WAIT_FOR_TAKEOFF and p.takeoff_held
    released_at = MAT + np.array([0.60, -0.40, 1.45])        # our step-out (and a route round the outside) moved it
    place(p, released_at)
    OnlineJoinPlanner.takeoff_gate_callback(p, RELEASE)
    assert p.phase == P.TAKEOFF and not p.takeoff_held
    assert p.takeoff_start_position == pytest.approx(released_at)


def test_while_held_his_mpc_is_given_a_hover_where_quad4_is():
    p = _gated()
    place(p, MAT + np.array([0.0, 0.0, 1.30]))
    assert p.wait_takeoff_hold() == pytest.approx(MAT)        # not held: the mat, as before
    command(p, 'TAKEOFF')
    here = MAT + np.array([0.30, 0.10, 1.40])
    place(p, here)
    assert p.wait_takeoff_hold() == pytest.approx(here)


def test_a_takeoff_after_the_release_goes_at_once_and_later_releases_change_nothing():
    p = _gated()
    place(p, MAT + np.array([0.20, 0.0, 1.40]))
    OnlineJoinPlanner.takeoff_gate_callback(p, RELEASE)
    assert p.phase == P.WAIT_FOR_TAKEOFF
    command(p, 'TAKEOFF')
    assert p.phase == P.TAKEOFF
    start = p.takeoff_start_position.copy()
    place(p, MAT + np.array([0.50, 0.0, 1.60]))
    OnlineJoinPlanner.takeoff_gate_callback(p, RELEASE)      # announced every 1 s
    assert p.phase == P.TAKEOFF and p.takeoff_start_position == pytest.approx(start)


def test_without_the_topic_a_takeoff_goes_at_once():
    p = floor_planner('floor_join')
    command(p, 'TAKEOFF')
    assert p.phase == P.TAKEOFF


def _stale_ring(p):
    p.last_attachment_pose_time = p.clock.t - 5.0              # his floor_join refusal: no fresh attachment pose


def test_a_refusal_in_the_air_holds_where_it_is_and_never_disarms():
    """The critic (5 Nov): at the gate his MPC may already fly QUAD4 through our mux; his refusal used to send DISARM."""
    p = _gated()
    place(p, MAT + np.array([0.0, 0.0, 1.30]))
    command(p, 'TAKEOFF')
    here = MAT + np.array([0.50, -0.30, 1.40])
    place(p, here)
    _stale_ring(p)
    OnlineJoinPlanner.takeoff_gate_callback(p, RELEASE)
    assert p.drone_command_publisher.messages == []           # no DISARM
    assert p.phase == P.WAIT_FOR_TAKEOFF and p.takeoff_held
    place(p, here + [0.10, 0.0, 0.0])                         # his MPC drifts: the hold does not follow it
    _stale_ring(p)
    assert p.wait_takeoff_hold() == pytest.approx(here)
    OnlineJoinPlanner.takeoff_gate_callback(p, RELEASE)       # still refused: still holding, still no DISARM
    assert p.phase == P.WAIT_FOR_TAKEOFF and p.drone_command_publisher.messages == []
    there = here + [0.10, 0.0, 0.0]
    place(p, there)                                           # the ring's pose is back
    OnlineJoinPlanner.takeoff_gate_callback(p, RELEASE)
    assert p.phase == P.TAKEOFF and p.takeoff_start_position == pytest.approx(there)
    assert p.air_hold_position is None


def test_a_refusal_on_the_mat_still_disarms_as_his_did():
    p = floor_planner('floor_join')
    p.build_reference()
    _stale_ring(p)
    command(p, 'TAKEOFF')
    assert p.phase == P.LANDED_DISARMED
    assert [m.data for m in p.drone_command_publisher.messages] == ['DISARM']
