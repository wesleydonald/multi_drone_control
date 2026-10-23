"""The fleet ARM gate (R0560): a drone that fails or never answers its arming call must
leave the fleet unarmed, disarm through the services (no fleet abort, Q6b), and make
TAKEOFF refuse."""
import types
import pytest
import fleet_manager.fleet_manager_node as m


class _Fut:
    def __init__(self, ok):
        self.ok = ok

    def done(self):
        return self.ok is not None

    def result(self):
        return types.SimpleNamespace(success=self.ok, message='')


class _Client:
    def __init__(self, ok, available=True):
        self.ok, self.available = ok, available

    def wait_for_service(self, timeout_sec=None):
        return self.available

    def call_async(self, req):
        return _Fut(self.ok)


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, s): self.lines.append(s)
    warn = warning = error = debug = info


def _fake(clients):
    f = types.SimpleNamespace(arming_clients=clients, fleet_armed=True, flying=False,
                              master_step=5, _log=_Log(), aborts=[], cmds=[],
                              mux_state={})
    f.get_logger = lambda: f._log
    f._disarm_fleet = lambda emergency=False, reason='', keep_alive=False: f.aborts.append((emergency, reason))
    f._publish_drone_command = lambda c: f.cmds.append(c)
    f.num_drones = len(clients)
    return f


@pytest.fixture(autouse=True)
def _no_spin(monkeypatch):
    monkeypatch.setattr(m.rclpy, 'spin_until_future_complete', lambda *a, **k: None)


def _arm(f):
    m.CentralController._arm_fleet_thread(f)


def test_all_arm_ok():
    f = _fake({i: _Client(True) for i in range(3)})
    _arm(f)
    assert f.fleet_armed and not f.aborts


@pytest.mark.parametrize('bad', [False, None])      # refused, or never answered
def test_one_drone_fails_blocks_takeoff(bad):
    f = _fake({0: _Client(bad), 1: _Client(True), 2: _Client(True)})
    _arm(f)
    assert not f.fleet_armed
    assert f.aborts and f.aborts[0][0] is False and '[1]' in f.aborts[0][1]
    m.CentralController._takeoff_fleet(f)
    assert f.cmds == [] and not f.flying


def test_service_missing_resets_a_previous_arm():
    f = _fake({0: _Client(True), 1: _Client(True, available=False)})
    f.fleet_armed = True                                 # an earlier good ARM
    _arm(f)
    assert not f.fleet_armed
    m.CentralController._takeoff_fleet(f)
    assert f.cmds == []


def _fc_fake(states):
    f = _fake({i: _Client(True) for i in range(len(states))})
    f.fc_arm_state = dict(enumerate(states))
    return f


def test_fc_gate_needs_every_fc_armed(monkeypatch):
    monkeypatch.setattr(m.time, 'sleep', lambda s: None)
    f = _fc_fake(['armed', 'armed', 'armed', 'armed'])
    assert m.CentralController._wait_fc_armed(f, timeout_s=0.05) is True and not f.aborts


@pytest.mark.parametrize('st', ['unconfirmed: no flight-mode frame since the arm edge',
                                "failed: FC says '!ERR*' after 2 tries", 'pending', ''])
def test_fc_gate_refuses_unconfirmed_failed_or_silent(monkeypatch, st):
    monkeypatch.setattr(m.time, 'sleep', lambda s: None)
    f = _fc_fake(['armed', st, 'armed', 'armed'])
    assert m.CentralController._wait_fc_armed(f, timeout_s=0.05) is False
    assert f.aborts and f.aborts[0][0] is False
    assert any('replug drone 2' in s and 'relaunch T2 only if a drone moved' in s for s in f._log.lines)


# ── 14 Oct: the partner's drone first, only the refused drones named, a DISARM stops the sequence, one burst ──

def _rig(clients, fc):
    f = _fake(clients)
    f.require_fc_armed, f.fc_arm_state = True, dict(fc)
    f._wait_fc_armed = lambda timeout_s=10.0, gen=None: m.CentralController._wait_fc_armed(f, 0.05, gen)
    return f


def test_partner_drone_not_armed_is_refused_at_once():
    f = _rig({i: _Client(True) for i in range(4)}, {3: 'disarmed'})
    f.mux_state = {3: 'partner'}
    _arm(f)
    assert any('ARM REFUSED: drone 4 flies the partner' in s for s in f._log.lines)
    assert f.aborts == []                       # nothing armed yet, nothing to disarm


def test_only_the_refused_drone_is_told_to_replug():
    f = _rig({i: _Client(True) for i in range(3)},
             {0: 'pending', 1: "failed: FC says '!ERR*' after 2 tries", 2: 'armed'})
    _arm(f)
    told = [s for s in f._log.lines if 'replug' in s]
    assert len(told) == 1 and 'drone 2' in told[0]
    assert any('still confirming' in s and '1' in s for s in f._log.lines)


def test_a_disarm_stops_the_arm_sequence_quietly():
    f = _rig({i: _Client(True) for i in range(3)}, {0: 'pending', 1: 'pending', 2: 'pending'})
    f._disarm_gen = 0
    real = m.CentralController._wait_fc_armed

    def disarmed_meanwhile(timeout_s=10.0, gen=None):
        f._disarm_gen += 1                      # the operator's DISARM arrives while the FCs confirm
        return real(f, 0.05, gen)
    f._wait_fc_armed = disarmed_meanwhile
    _arm(f)
    assert not any('replug' in s for s in f._log.lines) and f.aborts == []
    assert any('stopped by a DISARM' in s for s in f._log.lines)


def test_a_disarm_burst_reaches_the_radios_every_time_and_is_said_once(monkeypatch):
    sent, said, threads = [], [], []
    pub = types.SimpleNamespace(publish=sent.append)
    f = types.SimpleNamespace(abort_pub=pub, elrs_publishers={0: pub, 1: pub}, num_drones=2,
                              get_logger=lambda: _Log(), _disarm_fleet_thread=lambda: None)
    monkeypatch.setattr(m, '_announce', lambda node, level, text: said.append(text))
    monkeypatch.setattr(m.threading, 'Thread', lambda target, daemon: types.SimpleNamespace(
        start=lambda: threads.append(target)))
    for _ in range(5):
        m.CentralController._disarm_fleet(f, emergency=True, reason='operator DISARM')
    assert len(sent) == 5 * 3                   # abort + one stop per radio, every press
    assert len(said) == 1 and len(threads) == 1
