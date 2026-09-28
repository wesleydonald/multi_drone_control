"""Fleet-abort latch in the ELRS mux (card docs/experiments/2026-09-28_mux_abort_latch.md,
v3 item 3). Falsifier: any forwarded command armed or above idle throttle after an abort.
Node tests run on an isolated ROS domain and call the callbacks directly."""
import random
import time

import pytest

pytest.importorskip('interfaces.msg')
import rclpy  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from interfaces.msg import ELRSCommand  # noqa: E402

from drone_magnet.elrs_mux import ElrsMux, abort_latch_command  # noqa: E402

DOMAIN = 91


def _cmd(armed=True, thr=0.3, rates=(0.2, -0.1, 0.05), mag=0.7):
    m = ELRSCommand(armed=armed, channel_0=rates[0], channel_1=rates[1], channel_2=thr,
                    channel_3=rates[2])
    m.channel_10 = mag
    return m


def _is_latched_out(m):
    return (m.armed is False and m.channel_0 == 0.0 and m.channel_1 == 0.0
            and m.channel_2 == -1.0 and m.channel_3 == 0.0)


# ── the pure function, per message ───────────────────────────────────────────

def test_latch_disarms_idles_and_zeroes_rates_keeps_aux():
    src = _cmd()
    out = abort_latch_command(src)
    assert _is_latched_out(out)
    assert out.channel_10 == pytest.approx(0.7)             # magnet aux kept
    assert src.armed is True and src.channel_2 == pytest.approx(0.3)   # input untouched


def test_latch_with_no_command_seen_is_a_plain_disarm():
    assert _is_latched_out(abort_latch_command(None))


def test_latch_holds_for_any_message():
    rng = random.Random(7)
    for _ in range(500):
        m = _cmd(armed=rng.random() < 0.8, thr=rng.uniform(-1.0, 1.0),
                 rates=tuple(rng.uniform(-1.0, 1.0) for _ in range(3)),
                 mag=rng.uniform(-1.0, 1.0))
        assert _is_latched_out(abort_latch_command(m))


# ── the node ─────────────────────────────────────────────────────────────────

class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


@pytest.fixture
def mux_factory(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=['--ros-args', '-p', 'drone_id:=2', '-p', 'handoff_topic:=/fleet/handover'],
               domain_id=DOMAIN)
    nodes = []

    def make():
        n = ElrsMux()
        n.pub = _Rec()
        n.states = n._state_pub = _Rec()
        n.states.msgs.append(String(data=n._state))  # the one sent in __init__
        nodes.append(n)
        return n
    yield make
    for n in nodes:
        n.destroy_node()
    rclpy.shutdown()


def _states(n):
    return [m.data for m in n.states.msgs]


def test_no_abort_forwards_the_partner_untouched(mux_factory):
    n = mux_factory()
    n._tejen_cb(_cmd())
    out = n.pub.msgs[-1]
    assert out.armed and out.channel_2 == pytest.approx(0.3)
    assert _states(n) == ['partner']


def test_our_tracker_disarm_does_not_reach_the_radio_while_partner_selected(mux_factory):
    n = mux_factory()
    n._tejen_cb(_cmd())
    before = len(n.pub.msgs)
    n._diss_cb(ELRSCommand(armed=False, channel_2=-1.0))    # the tracker's safety disarm
    assert len(n.pub.msgs) == before


def test_abort_before_handover_disarms_at_once_and_holds_against_the_partner(mux_factory):
    n = mux_factory()
    n._tejen_cb(_cmd())
    n._abort_cb(String(data='operator ESTOP'))
    assert _is_latched_out(n.pub.msgs[-1])                  # immediate disarm
    assert n.pub.msgs[-1].channel_10 == pytest.approx(0.7)
    assert _states(n)[-1] == 'latched'
    k = len(n.pub.msgs)
    for _ in range(10):
        n._tejen_cb(_cmd(thr=0.6))                           # he keeps flying armed
    assert len(n.pub.msgs) == k + 10
    assert all(_is_latched_out(m) for m in n.pub.msgs[k:])


def test_abort_after_handover_holds_our_stream_too(mux_factory):
    n = mux_factory()
    n._handoff_cb(Bool(data=True))
    n._diss_cb(_cmd(thr=0.4))                                # live -> handed over
    assert _states(n)[-1] == 'ours' and n.pub.msgs[-1].armed
    n._abort_cb(String(data='drone 0 disarmed unexpectedly'))
    n._diss_cb(_cmd(thr=0.4))
    n._tejen_cb(_cmd(thr=0.4))                               # not selected: dropped
    assert all(_is_latched_out(m) for m in n.pub.msgs[-2:])
    assert _states(n) == ['partner', 'ours', 'latched']


def test_latched_state_survives_a_later_handover_or_release(mux_factory):
    n = mux_factory()
    n._abort_cb(String(data='x'))
    n._handoff_cb(Bool(data=True))
    n._diss_cb(_cmd())
    assert _states(n)[-1] == 'latched'
    assert _is_latched_out(n.pub.msgs[-1])


def test_second_abort_is_a_no_op(mux_factory):
    n = mux_factory()
    n._abort_cb(String(data='a'))
    t = n._latch_timer
    n._abort_cb(String(data='b'))
    assert n._latch_timer is t and _states(n).count('latched') == 1


def test_latched_mux_republishes_the_disarm_on_its_timer(mux_factory):
    n = mux_factory()
    n._abort_cb(String(data='x'))
    k = len(n.pub.msgs)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 0.5:
        rclpy.spin_once(n, timeout_sec=0.02)
    new = n.pub.msgs[k:]
    assert 6 <= len(new) <= 14                              # 20 Hz over 0.5 s
    assert all(_is_latched_out(m) for m in new)


def test_restart_clears_the_latch(mux_factory):
    a = mux_factory()
    a._abort_cb(String(data='x'))
    b = mux_factory()                                        # a relaunched mux
    b._tejen_cb(_cmd())
    assert b.pub.msgs[-1].armed and _states(b) == ['partner']


def test_abort_latch_false_ignores_the_abort(mux_factory):
    n = mux_factory()
    n.set_parameters([Parameter('abort_latch', value=False)])
    n._abort_cb(String(data='x'))
    n._tejen_cb(_cmd())
    assert n.pub.msgs[-1].armed and _states(n) == ['partner'] and n._latch_timer is None


def test_ours_only_mux_reports_ours(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=['--ros-args', '-p', 'drone_id:=2', '-p', 'approach_stream:=false'],
               domain_id=DOMAIN)
    try:
        n = ElrsMux()
        assert n._state == 'ours'
        n.destroy_node()
    finally:
        rclpy.shutdown()
