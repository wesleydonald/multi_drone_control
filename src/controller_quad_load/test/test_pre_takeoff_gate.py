"""A tracker that disarms between ARM and TAKEOFF grounds our fleet like a failed ARM
(Wesley Q11, 2026-09-29; card docs/experiments/2026-09-29_pre_takeoff_disarm_gate.md v2):
service disarm, no /fleet/abort, no direct ELRS, TAKEOFF refused. R0749: every tracker
pose-timed-out between ARM and TAKEOFF and the manager still took TAKEOFF."""
import controller_quad_load.main as m
from std_msgs.msg import Bool

from test_mux_abort_manager import _Client, _direct_elrs, _fake, sync  # noqa: F401


def _armed(f):
    """ARM completed and every tracker reported True (the normal pre-TAKEOFF state)."""
    f._arm_fleet_thread()
    for i in range(f.num_drones):
        f._arming_feedback_callback(Bool(data=True), i)
    assert f.fleet_armed


def _grounded_quietly(f):
    return (f.service_disarms == [True] and f.abort_pub.msgs == [] and _direct_elrs(f) == 0
            and not f.fleet_armed and f.shutdown_requested)


def test_disarm_between_arm_and_takeoff_grounds_the_fleet(sync):
    f = _fake()
    _armed(f)
    f._arming_feedback_callback(Bool(data=False), 2)
    assert _grounded_quietly(f)
    assert any('Drone 2 disarmed before TAKEOFF' in s for s in f._log.lines)
    f._takeoff_fleet()
    assert f.cmds == [] and not f.flying


def test_r0749_every_tracker_drops_fires_once(sync):
    f = _fake()
    _armed(f)
    for i in range(4):
        f._arming_feedback_callback(Bool(data=False), i)
    assert f.service_disarms == [True] and f.abort_pub.msgs == []


def test_the_fleets_own_disarm_does_not_re_enter(sync):
    f = _fake()
    _armed(f)
    f._arming_feedback_callback(Bool(data=False), 0)
    for i in range(1, 4):                      # the service disarm's own Falses
        f._arming_feedback_callback(Bool(data=False), i)
    assert f.service_disarms == [True]


def test_false_before_fleet_armed_is_caught_at_takeoff(sync):
    # (a) True then False land while the ARM thread still runs: fleet_armed is set after
    f = _fake()
    f._arming_feedback_callback(Bool(data=True), 1)
    f._arming_feedback_callback(Bool(data=False), 1)
    f._arm_fleet_thread()
    for i in (0, 2, 3):
        f._arming_feedback_callback(Bool(data=True), i)
    assert f.fleet_armed and f.service_disarms == []
    f._takeoff_fleet()
    assert f.cmds == [] and _grounded_quietly(f)
    assert any('TAKEOFF REFUSED: drone(s) [1] not armed' in s for s in f._log.lines)


def test_late_true_then_takeoff_is_accepted(sync):
    # (b) the service returned before its True arrived: no fire, TAKEOFF goes
    f = _fake()
    f._arm_fleet_thread()
    for i in range(4):
        f._arming_feedback_callback(Bool(data=True), i)
    f._takeoff_fleet()
    assert f.cmds == ['TAKEOFF'] and f.flying and f.service_disarms == []


def test_in_flight_disarm_keeps_the_emergency_path(sync):
    f = _fake()
    _armed(f)
    f._takeoff_fleet()
    f._arming_feedback_callback(Bool(data=False), 1)
    assert len(f.abort_pub.msgs) == 1 and _direct_elrs(f) == 4


def test_disarm_before_arm_or_after_land_does_nothing(sync):
    f = _fake()
    f._arming_feedback_callback(Bool(data=True), 0)
    f._arming_feedback_callback(Bool(data=False), 0)       # before ARM
    assert f.service_disarms == [] and not f.shutdown_requested


def test_missing_arming_service_disarms_the_fleet(sync):
    f = _fake()

    class _NoService(_Client):
        def wait_for_service(self, timeout_sec=None):
            return False
    f.arming_clients[3] = _NoService(True)
    f._arm_fleet_thread()
    assert _grounded_quietly(f)
    f._takeoff_fleet()
    assert f.cmds == []


def test_disarms_after_land_do_nothing_more(sync):
    f = _fake()
    _armed(f)
    f._takeoff_fleet()
    f._landed_callback = m.CentralController._landed_callback.__get__(f)
    f._land_fleet = m.CentralController._land_fleet.__get__(f)
    f._land_fleet()
    f._landed_callback(Bool(data=True))
    for i in range(4):                                     # the landing disarm's Falses
        f._arming_feedback_callback(Bool(data=False), i)
    assert f.service_disarms == [True] and f.abort_pub.msgs == [] and _direct_elrs(f) == 0


def test_repeat_arm_then_takeoff_is_accepted(sync):
    # the M2 driver re-sends ARM until every /ours feedback is True
    f = _fake()
    _armed(f)
    _armed(f)
    f._takeoff_fleet()
    assert f.cmds == ['TAKEOFF'] and f.service_disarms == []


def test_before_takeoff_a_drone_on_the_partner_stream_grounds_our_fleet_too(sync):
    # deliberately stricter than the in-flight F5 scoping: before TAKEOFF nothing of ours
    # flies, and a dead tracker of ours would make the hand-over a mixed-control fleet
    f = _fake(mux_state={3: 'partner'})
    _armed(f)
    f._arming_feedback_callback(Bool(data=False), 3)
    assert _grounded_quietly(f)
