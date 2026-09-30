"""Fleet manager side of the mux abort latch (card 2026-09-28_mux_abort_latch.md v3):
1 ARM failed -> service disarm only; 2 a tracker fault behind a mux that forwards the
partner does not escalate; 4 ARM refused while a mux is latched. Plus the mux_state QoS
and a live late-join check on an isolated ROS domain."""
import time
import types

import pytest

import controller_quad_load.main as m
from std_msgs.msg import Bool, String

DOMAIN = 91


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, s): self.lines.append(s)
    warn = warning = error = debug = info


class _Fut:
    def __init__(self, ok):
        self.ok = ok

    def done(self):
        return self.ok is not None

    def result(self):
        return types.SimpleNamespace(success=self.ok, message='')


class _Client:
    def __init__(self, ok):
        self.ok, self.calls = ok, 0

    def wait_for_service(self, timeout_sec=None):
        self.calls += 1
        return True

    def call_async(self, req):
        self.calls += 1
        return _Fut(self.ok)

    def service_is_ready(self):
        return True


class _SyncThread:
    def __init__(self, target, daemon=None):
        self.target = target

    def start(self):
        self.target()


def _fake(n=4, ok=True, mux_state=None):
    f = types.SimpleNamespace(
        num_drones=n, arming_clients={i: _Client(ok) for i in range(n)},
        fleet_armed=False, flying=False, landing=False, shutdown_requested=False,
        master_step=0, mux_state=dict(mux_state or {}),
        drone_armed={i: False for i in range(n)}, drone_last_feedback={},
        _wall_clock=types.SimpleNamespace(now=lambda: 0), _log=_Log(),
        abort_pub=_Rec(), elrs_publishers={i: _Rec() for i in range(n)},
        service_disarms=[], cmds=[])
    f.get_logger = lambda: f._log
    C = m.CentralController
    for name in ('_disarm_fleet', '_takeoff_fleet', '_arm_fleet_thread',
                 '_arming_feedback_callback', '_mux_state_callback'):
        setattr(f, name, getattr(C, name).__get__(f))
    f._disarm_fleet_thread = lambda: f.service_disarms.append(True)
    f._publish_drone_command = lambda c: f.cmds.append(c)
    return f


@pytest.fixture
def sync(monkeypatch):
    """Run the manager's worker threads inline (module-level patches: fake-node tests only)."""
    monkeypatch.setattr(m.threading, 'Thread', _SyncThread)
    monkeypatch.setattr(m.rclpy, 'spin_until_future_complete', lambda *a, **k: None)


def _direct_elrs(f):
    return sum(len(p.msgs) for p in f.elrs_publishers.values())


# ── item 1: ARM failed ───────────────────────────────────────────────────────

def test_arm_failed_disarms_through_the_services_only(sync):
    f = _fake(ok=True)
    f.arming_clients[2] = _Client(False)
    f._arm_fleet_thread()
    assert f.service_disarms == [True]
    assert f.abort_pub.msgs == [] and _direct_elrs(f) == 0
    assert f.shutdown_requested and not f.fleet_armed
    f._takeoff_fleet()
    assert f.cmds == [] and not f.flying


def test_operator_estop_still_aborts_and_disarms_directly(sync):
    f = _fake()
    f._disarm_fleet(emergency=True, reason='operator ESTOP')
    assert [x.data for x in f.abort_pub.msgs] == ['operator ESTOP']
    assert _direct_elrs(f) == 4 and f.service_disarms == [True]


def test_operator_disarm_latches_the_muxes_through_fleet_abort(sync):
    # the rig kill switch is DISARM: it must reach drones still on the partner's stream
    f = _fake(mux_state={3: 'partner'})
    f._command_callback = m.CentralController._command_callback.__get__(f)
    f.flying = True
    f._command_callback(String(data='disarm'))
    assert [x.data for x in f.abort_pub.msgs] == ['operator DISARM']
    # the fast path too: a disarm straight to every radio (reaches a wedged tracker's drone)
    assert _direct_elrs(f) == 4
    assert f.service_disarms == [True] and not f.flying


# ── item 2: fault scoping ────────────────────────────────────────────────────

def _flying(f, drone):
    f.flying = True
    f.drone_armed[drone] = True


def test_fault_behind_a_partner_mux_does_not_escalate(sync):
    f = _fake(mux_state={3: 'partner'})
    _flying(f, 3)
    f._arming_feedback_callback(Bool(data=False), 3)
    assert f.abort_pub.msgs == [] and _direct_elrs(f) == 0 and f.service_disarms == []
    assert f.flying                                          # the fleet keeps flying
    assert any('not escalating' in s for s in f._log.lines)


@pytest.mark.parametrize('state', ['ours', None])            # handed over / no mux at all
def test_fault_of_a_commanded_drone_grounds_everyone(state, sync):
    f = _fake(mux_state={} if state is None else {3: state})
    _flying(f, 3)
    f._arming_feedback_callback(Bool(data=False), 3)
    assert len(f.abort_pub.msgs) == 1 and _direct_elrs(f) == 4


def test_partner_scope_is_per_drone(sync):
    f = _fake(mux_state={3: 'partner'})
    _flying(f, 1)
    f._arming_feedback_callback(Bool(data=False), 1)          # a carrier we fly
    assert len(f.abort_pub.msgs) == 1


# ── item 4: ARM gate ─────────────────────────────────────────────────────────

def test_arm_refused_while_a_mux_is_latched_before_any_service_call(sync):
    f = _fake(mux_state={0: 'ours', 3: 'latched'})
    f.fleet_armed = True
    f._arm_fleet_thread()
    assert 'ARM REFUSED: mux latched on drone 3 (relaunch the muxes)' in f._log.lines
    assert all(c.calls == 0 for c in f.arming_clients.values())
    assert not f.fleet_armed and f.service_disarms == [] and f.abort_pub.msgs == []
    f._takeoff_fleet()
    assert f.cmds == []


@pytest.mark.parametrize('states', [{}, {3: 'partner'}, {0: 'ours', 3: 'ours'}])
def test_arm_proceeds_without_a_latched_mux(states, sync):
    f = _fake(mux_state=states)
    f._arm_fleet_thread()
    assert f.fleet_armed and not any('REFUSED' in s for s in f._log.lines)


def test_mux_state_callback_normalises(sync):
    f = _fake()
    f._mux_state_callback(String(data=' Latched\n'), 2)
    assert f.mux_state == {2: 'latched'}


# ── mux_state QoS, both ends ─────────────────────────────────────────────────

def test_mux_state_qos_is_compatible_and_latched():
    mux = pytest.importorskip('drone_magnet.elrs_mux')
    from rclpy.qos import (DurabilityPolicy, QoSCompatibility, ReliabilityPolicy,
                           qos_check_compatible)
    # WARNING only flags the unset (system default) liveliness on both ends
    ok, why = qos_check_compatible(mux.MUX_STATE_QOS, m.MUX_STATE_QOS)
    assert ok != QoSCompatibility.ERROR, why
    for q in (mux.MUX_STATE_QOS, m.MUX_STATE_QOS):
        assert q.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert q.reliability == ReliabilityPolicy.RELIABLE


def test_live_late_manager_reads_latch_and_refuses_arm(monkeypatch):
    """A mux started first, a manager started after it (the split launch), then an abort:
    the late manager reads 'partner', then 'latched', and refuses ARM. Isolated domain."""
    mux_mod = pytest.importorskip('drone_magnet.elrs_mux')
    import rclpy
    from interfaces.msg import ELRSCommand
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setattr(m.rclpy, 'spin_until_future_complete', lambda *a, **k: None)
    rclpy.init(args=['--ros-args', '-p', 'drone_id:=2', '-p', 'num_drones:=3'],
               domain_id=DOMAIN)
    try:
        mux = mux_mod.ElrsMux()
        helper = rclpy.create_node('abort_test_helper')
        out = []
        helper.create_subscription(ELRSCommand, '/drone_2/ELRSCommand', out.append, 10)
        abort = helper.create_publisher(String, '/fleet/abort', 5)
        tejen = helper.create_publisher(ELRSCommand, '/drone_2/ELRSCommand_tejen', 1)
        mgr = m.CentralController()                          # joins after the mux
        ex = rclpy.executors.SingleThreadedExecutor()
        for n in (mux, helper, mgr):
            ex.add_node(n)

        def spin_until(pred, t=5.0):
            t0 = time.monotonic()
            while time.monotonic() - t0 < t and not pred():
                ex.spin_once(timeout_sec=0.05)
            return pred()

        assert spin_until(lambda: mgr.mux_state.get(2) == 'partner')
        assert spin_until(lambda: abort.get_subscription_count() >= 1)
        abort.publish(String(data='operator ESTOP'))
        assert spin_until(lambda: mgr.mux_state.get(2) == 'latched')
        n0 = len(out)
        for _ in range(5):
            tejen.publish(ELRSCommand(armed=True, channel_2=0.5))
            spin_until(lambda: False, 0.05)
        assert spin_until(lambda: len(out) > n0 + 3)
        assert all((not c.armed) and c.channel_2 == -1.0 for c in out)
        logs = []
        mgr.get_logger = lambda: types.SimpleNamespace(
            info=logs.append, warn=logs.append, error=logs.append)
        mgr._arm_fleet_thread()
        assert 'ARM REFUSED: mux latched on drone 2 (relaunch the muxes)' in logs
        assert not mgr.fleet_armed
        for n in (mux, helper, mgr):
            n.destroy_node()
    finally:
        rclpy.shutdown()
