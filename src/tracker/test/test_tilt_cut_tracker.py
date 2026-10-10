"""The drone-3 tracker's tilt fault (card 2026-10-11_quad4_tilt_cut, critic round 1 C1): evaluated on every fresh
pose on its own subscription, ahead of the pose-timeout and armed gates and of the envelope's latch. A disarm
clears armed and takeoff_requested on the same tick, and a knock latches the speed fault first: the fault must
still be published. ARM clears it; speed is only logged (M2)."""
import math
import types

import pytest

pytest.importorskip('interfaces.msg')
from interfaces.msg import MotionCaptureState  # noqa: E402

from tracker.tracker_node import Controller  # noqa: E402
from utility_objects.tilt_cut import TiltCutConfig, TiltCutRule  # noqa: E402

HZ = 120.0


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m.data)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = warning = error = info


def _node(in_authority=False):
    log = _Log()
    clock = [100.0]
    c = types.SimpleNamespace(
        drone_id=3, armed=False, takeoff_requested=False, _in_authority=in_authority,
        _tilt_rule=TiltCutRule(TiltCutConfig()), _tilt_prev_armed=False, _tilt_glitches_said=0,
        _tilt_speed_said=False, _tilt_pub=_Pub(), tilt_fault_topic='/drone_3/tilt_fault',
        envelope=types.SimpleNamespace(limits=types.SimpleNamespace(max_speed=3.0)))
    c.get_logger = lambda: log
    c.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(clock[0] * 1e9)))
    c.clock, c.log = clock, log
    c.pose = types.MethodType(Controller._tilt_pose_cb, c)
    return c


def _msg(deg, speed=0.0):
    m = MotionCaptureState()
    h = math.radians(deg) / 2.0
    m.pose.orientation.w, m.pose.orientation.x = math.cos(h), math.sin(h)
    m.twist.linear.x = float(speed)
    return m


def _fly(c, degs, speed=0.0):
    for d in degs:
        c.pose(_msg(d, speed))
        c.clock[0] += 1.0 / HZ


def _arm_and_takeoff(c):
    c.armed = True
    _fly(c, [0.0])
    c.takeoff_requested = True
    _fly(c, [0.0])


def test_same_tick_crossing_the_envelope_disarm_does_not_stop_the_rule():
    """C1(a): the envelope faults and cb.disarm clears armed and takeoff_requested on the tick the tilt crosses 60."""
    c = _node()
    _arm_and_takeoff(c)
    _fly(c, [40, 55])
    c.armed = c.takeoff_requested = False                       # the envelope's disarm, same tick
    _fly(c, [61, 62, 63])
    assert c._tilt_pub.msgs == [True]
    assert any('TILT FAULT (tilt)' in ln and 'another stream' in ln and 'disarmed' in ln for ln in c.log.lines)


def test_speed_latches_first_then_the_tilt_still_faults():
    """C1(b): a knock trips the envelope's speed rule at 0.25 s (R1263/R1287); the tilt passes 60 deg later."""
    c = _node()
    _arm_and_takeoff(c)
    _fly(c, [10, 15], speed=4.6)
    c.armed = c.takeoff_requested = False                       # ENVELOPE FAULT: speed
    _fly(c, [20 + 3.3 * k for k in range(20)], speed=6.0)
    assert c._tilt_pub.msgs == [True]
    assert sum('would cut (speed)' in ln for ln in c.log.lines) == 1   # logged once, not a cut


def test_once_per_flight_and_arm_clears_it():
    c = _node()
    _arm_and_takeoff(c)
    _fly(c, [20, 40, 60, 80] + [90] * 10)                       # a tumble, 20 deg a pose
    assert c._tilt_pub.msgs == [True]
    c.armed = c.takeoff_requested = False
    _fly(c, [90] * 5)                                           # lying tipped: no repeat
    assert c._tilt_pub.msgs == [True]
    c.armed = True                                              # the next ARM: a new flight, nothing latched
    _fly(c, [0.0])
    assert c._tilt_rule.fault is None and not c._tilt_rule.flying


def test_nothing_before_takeoff_and_a_glitch_is_announced():
    c = _node()
    c.armed = True
    _fly(c, [90] * 10)                                          # armed on its mat, never TAKEOFF
    assert c._tilt_pub.msgs == []
    c.takeoff_requested = True
    _fly(c, [0, 0, 80, 80, 80])                                 # a one-pose step of 80 deg: a mis-solve
    assert c._tilt_pub.msgs == [] and any('mocap glitch' in ln for ln in c.log.lines)


def test_ours_flying_is_named_in_the_line():
    c = _node(in_authority=True)
    _arm_and_takeoff(c)
    _fly(c, [30, 50, 61, 62, 63])
    assert c._tilt_pub.msgs == [True] and any("ours flying it" in ln for ln in c.log.lines)


# ── warm_fault_not_live (rig-port critic F6): a fault while another stream flies it: not live, not latched ──────

from interfaces.msg import ELRSCommand  # noqa: E402
from utility_objects.safety import EnvelopeChecker, EnvelopeLimits  # noqa: E402


def _warm(in_authority=False, on=True):
    clock = [100.0]
    c = types.SimpleNamespace(
        drone_id=3, armed=True, _was_armed=True, takeoff_requested=True, _in_authority=in_authority,
        warm_fault_not_live=on, _warm_hold_until=None, safety_enabled=True, _aborted=False, _last_ref_wall=None,
        payload_quat=None, _safety_warn_ctr=0, envelope=EnvelopeChecker(EnvelopeLimits()), disarms=[], sent=[],
        current_pose=None)
    log = _Log()
    c.get_logger = lambda: log
    c.log = log
    c._safety_clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=int(clock[0] * 1e9)))
    c.clock = clock
    c.cb = types.SimpleNamespace(cmd_publisher_=types.SimpleNamespace(publish=c.sent.append))
    c._payload_checks_down = lambda: True
    c._do_safety_disarm = lambda reason='': c.disarms.append(reason)
    for name in ('_warm_unlatched', '_publish_warm_idle', '_warm_not_live', '_safety_check'):
        setattr(c, name, types.MethodType(getattr(Controller, name), c))
    return c


def _pose_deg(c, deg):
    h = math.radians(deg) / 2.0
    c.current_pose = [0.0, 0.0, 1.5, math.cos(h), math.sin(h), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def test_a_warm_fault_is_not_live_and_not_latched_then_live_again():
    c = _warm()
    _pose_deg(c, 90.0)
    for _ in range(5):                                         # past the 3-sample debounce
        c._safety_check()
        c.clock[0] += 0.02
    assert c.disarms == [] and not c._aborted
    assert c.sent and all(isinstance(m, ELRSCommand) and m.armed and m.channel_2 == -1.0 for m in c.sent)
    assert sum('not live' in ln for ln in c.log.lines) == 1  # one line per episode
    _pose_deg(c, 2.0)
    c.clock[0] += 0.3
    assert c._safety_check() is True                           # still held 0.5 s after the last fault
    c.clock[0] += 0.3
    assert c._safety_check() is False                          # live again: the caller flies the MPC


def test_with_authority_or_off_a_fault_latches_as_before():
    for c in (_warm(in_authority=True), _warm(on=False)):
        _pose_deg(c, 90.0)
        for _ in range(5):
            c._safety_check()
        assert c.disarms and c.disarms[0].startswith('drone tilt')
