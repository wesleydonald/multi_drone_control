"""A drone off the ring and down at its own rest is disarmed alone, an expected disarm (card
2026-10-09_departed_disarm_alone; Wesley 9 Oct: no armed idle at zero throttle after a flight)."""
import pytest

import fleet_manager.fleet_manager_node as m
from std_msgs.msg import Bool, String
from test_mux_abort_manager import _Client, _direct_elrs, _fake, sync  # noqa: F401


def _flying(f, *drones):
    f.flying = True
    for d in drones:
        f.drone_armed[d] = True


def _down(f, d):
    f._departed_callback(Bool(data=True), d)
    f._down_alone_callback(Bool(data=True), d)


@pytest.mark.parametrize("on_load", [{}, {3: False}])    # freed carrier / aborted newcomer
def test_down_at_its_rest_off_the_ring_disarms_that_drone_alone(on_load, sync):
    f = _fake(on_load=on_load)
    _flying(f, 0, 1, 2, 3)
    _down(f, 3)
    assert f.arming_clients[3].calls == 1 and all(f.arming_clients[i].calls == 0 for i in range(3))
    # after the reply, a disarm straight to its radio too, to that drone only
    assert _direct_elrs(f) == 1 and f.elrs_publishers[3].msgs[0].armed is False
    f._arming_feedback_callback(Bool(data=False), 3)        # its expected disarm comes back
    assert f.abort_pub.msgs == [] and _direct_elrs(f) == 1 and f.service_disarms == []
    assert f.fleet_cmd_pub.msgs == [] and f.flying            # no LAND, no stop: the ring flies on
    f._down_alone_callback(Bool(data=True), 3)                # a repeat asks nothing more
    assert f.arming_clients[3].calls == 1


def test_not_released_by_the_planner_is_not_disarmed(sync):
    f = _fake()
    _flying(f, 3)
    f._down_alone_callback(Bool(data=True), 3)
    assert f.arming_clients[3].calls == 0 and f._down_alone == set()
    f._arming_feedback_callback(Bool(data=False), 3)          # a disarm then is still a fault
    assert len(f.abort_pub.msgs) == 1


def test_a_latched_flag_from_before_this_flight_does_not_count(sync):
    f = _fake()
    f._departed_callback(Bool(data=True), 3)                  # arrives at subscription, on the floor
    _flying(f, 3)
    f._down_alone_callback(Bool(data=True), 3)
    assert f.arming_clients[3].calls == 0


def test_a_drone_on_the_partner_stream_is_not_disarmed(sync):
    f = _fake(mux_state={3: 'partner'})
    _flying(f, 3)
    _down(f, 3)
    assert f.arming_clients[3].calls == 0


def test_not_after_the_flight_or_with_its_tracker_gone(sync):
    f = _fake()
    _down(f, 3)                                               # not flying
    assert f.arming_clients[3].calls == 0
    f.arming_clients[3].service_is_ready = lambda: False
    _flying(f, 3)
    _down(f, 3)
    assert f.arming_clients[3].calls == 0 and f._down_alone == set()


def test_arm_clears_both_for_the_next_flight(sync):
    f = _fake()
    f._off_ring.add(3)
    f._down_alone.add(3)
    f._arm_fleet = lambda: None
    m.CentralController._command_callback(f, String(data='ARM'))
    assert f._off_ring == set() and f._down_alone == set()


def test_a_carrier_fault_after_a_drone_went_down_alone_still_grounds_everyone(sync):
    f = _fake()
    _flying(f, 0, 3)
    _down(f, 3)
    f._arming_feedback_callback(Bool(data=False), 0)
    assert len(f.abort_pub.msgs) == 1 and _direct_elrs(f) == 1 + 4


def test_the_fleet_disarm_still_reaches_a_drone_down_alone(sync):
    """Its tracker stays up after the disarm alone (R1378), so the final fleet disarm closes it with the others."""
    f = _fake()
    f._disarm_fleet_thread = m.CentralController._disarm_fleet_thread.__get__(f)
    f.get_logger = lambda: f._log
    f._down_alone.add(3)
    f._disarm_fleet_thread()
    assert all(f.arming_clients[i].calls == 1 for i in range(4))
    assert not any('not answering' in s for s in f._log.lines)


# ── LAND before TAKEOFF (Wesley 9 Oct: LAND lands every drone, whatever it is doing; R1311) ──

def test_land_before_takeoff_disarms_the_armed_fleet_and_stays_up(sync):
    f = _fake()
    f.fleet_armed = True
    m.CentralController._land_fleet(f)
    assert f.service_disarms == [True] and f.abort_pub.msgs == [] and _direct_elrs(f) == 0
    assert not f.shutdown_requested and not f.fleet_armed and not f.flying


def test_land_with_nothing_armed_changes_nothing(sync):
    f = _fake()
    m.CentralController._land_fleet(f)
    assert f.service_disarms == []

