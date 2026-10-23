"""The twin's per-channel command delay (card docs/experiments/2026-10-23_pair_mode_timing.md): the throttle and the
three rates each on their own delay from cmd_delay_from_s; cmd_delay_s alone keeps the single delay of 22 Oct, and
the defaults leave today's twin as it was."""
import collections

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.time import Time  # noqa: E402
from interfaces.msg import ELRSCommand  # noqa: E402
from simulation_communication.payload_betaflight_comm import PayloadBetaflightComm, due  # noqa: E402
from simulation_communication.rate_pid import RatePid  # noqa: E402

ENV = ('SIM_CMD_DELAY_S', 'SIM_CMD_DELAY_FROM_S', 'SIM_CMD_DELAY_THR_S', 'SIM_CMD_DELAY_ATT_S', 'SIM_RATE_KI')


class Clock:
    def __init__(self, t=0.0):
        self.t = t

    def now(self):
        return Time(nanoseconds=int(round(self.t * 1e9)))


@pytest.fixture
def make_node(monkeypatch):
    """make(SIM_...=value): a node built with those switches in its environment and none of the others."""
    rclpy.init()
    nodes = []

    def make(**env):
        for k in ENV:
            monkeypatch.delenv(k, raising=False)
        for k, v in env.items():
            monkeypatch.setenv(k, str(v))
        node = PayloadBetaflightComm()
        node.clock = Clock()
        node.get_clock = lambda: node.clock
        nodes.append(node)
        return node
    yield make
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def cmd(roll=0.0, pitch=0.0, thr=0.0, yaw=0.0, armed=True):
    m = ELRSCommand()
    m.armed, m.channel_0, m.channel_1, m.channel_2, m.channel_3 = armed, roll, pitch, thr, yaw
    return m


def send(node, t, msg):
    node.clock.t = t
    node._cmd_cb(msg)


def at(node, t):
    node.clock.t = t
    node._apply_due_cmds()
    return list(node.set_point) if node.set_point is not None else None


def test_due_pops_what_has_waited_long_enough_and_returns_the_latest():
    q = collections.deque([(0.00, 'a'), (0.02, 'b'), (0.04, 'c')])
    assert due(q, 0.034, 0.035) is None and len(q) == 3
    assert due(q, 0.035, 0.035) == (0.00, 'a')
    assert due(q, 0.076, 0.035) == (0.04, 'c') and not q
    assert due(collections.deque([(1.0, 'x')]), 1.0, 0.0) == (1.0, 'x')


def test_the_defaults_leave_todays_twin_unchanged(make_node):
    node = make_node()
    assert (node._d_thr, node._d_att, node._cmd_delay_from) == (0.0, 0.0, 0.0)
    send(node, 30.0, cmd(0.1, -0.2, 0.3, 0.05))
    sp = list(node.set_point)
    assert not node._att_queue and not node._thr_queue
    assert np.allclose([sp[0], sp[1], sp[3]], [node._betaflight_rates(0.1), node._betaflight_rates(-0.2),
                                               node._betaflight_rates(-0.05)])
    # the motor speeds are the old loop's: the PI on the rate error around the throttle, nothing else
    pid = RatePid(kp=0.5, ki=5.0)
    w = np.array([3.0, -1.0, 0.5])
    for _ in range(5):
        speeds = node._compute_motor_speeds(w, 0.001)
        off = pid.step(np.array([sp[0], sp[1], sp[3]]) - w, 0.001, node._active)
    old = np.clip([sp[2] - off[0] + off[1] + off[2], sp[2] - off[0] - off[1] - off[2],
                   sp[2] + off[0] + off[1] - off[2], sp[2] + off[0] - off[1] + off[2]], 0, 4631)
    assert np.allclose(speeds, old)


def test_cmd_delay_alone_delays_all_four_channels_together(make_node):
    node = make_node(SIM_CMD_DELAY_S=0.06, SIM_CMD_DELAY_FROM_S=24.0)
    assert node._d_thr == node._d_att == 0.06
    send(node, 23.98, cmd(0.0, 0.0, 0.0))          # before the from-time: straight into effect
    first = at(node, 23.98)
    send(node, 24.00, cmd(0.5, 0.0, 0.4))
    assert at(node, 24.059) == first
    late = at(node, 24.060)
    assert late[0] > 0 and late[2] > first[2]      # rates and throttle arrive in the same tick


def test_the_throttle_and_the_rates_each_wait_their_own_delay(make_node):
    node = make_node(SIM_CMD_DELAY_THR_S=0.035, SIM_CMD_DELAY_FROM_S=24.0)
    assert (node._d_thr, node._d_att) == (0.035, 0.0)
    send(node, 24.00, cmd(0.0, 0.0, 0.0))
    assert node.set_point is None                  # no throttle in effect yet: no motor command
    first_thr = at(node, 24.035)[2]
    send(node, 24.10, cmd(0.5, 0.0, 0.4))
    now = at(node, 24.10)
    assert now[0] == pytest.approx(node._betaflight_rates(0.5)) and now[2] == first_thr
    assert at(node, 24.134)[2] == first_thr
    assert at(node, 24.135)[2] > first_thr

    node = make_node(SIM_CMD_DELAY_S=0.02, SIM_CMD_DELAY_ATT_S=0.03)
    assert (node._d_thr, node._d_att) == (0.02, 0.05)
    send(node, 1.0, cmd(0.0, 0.0, 0.2))
    send(node, 1.01, cmd(0.5, 0.0, 0.4))
    sp = at(node, 1.03)
    assert sp is None                              # the throttle is due, the rates not yet
    sp = at(node, 1.05)
    assert sp[0] == 0.0 and sp[2] > 0              # the first rates, the second throttle
    assert at(node, 1.06)[0] == pytest.approx(node._betaflight_rates(0.5))


def test_the_switches_come_from_the_environment(make_node):
    node = make_node(SIM_CMD_DELAY_THR_S='0.035', SIM_CMD_DELAY_FROM_S='24.0')
    assert (node._d_thr, node._d_att, node._cmd_delay_from) == (0.035, 0.0, 24.0)
