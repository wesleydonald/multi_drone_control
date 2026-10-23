"""The mux's take-back input (combined M1 contact attach, card 2026-10-10_combined_m1_contact): with no attach our
planner takes QUAD4 back on /drone_3/take_back; the mux switches to our tracker latched, through the same
live-takeover rule as the weld. take_back_topic '' (the default) keeps today's mux. Node tests run on an isolated
ROS domain and call the callbacks directly."""
import pytest

from drone_magnet.handover_policy import HandoverPolicy


def test_take_back_waits_for_our_tracker_to_fly_then_latches():
    p = HandoverPolicy()
    assert p.take_back(now=0.0) is False                      # our tracker silent: nothing switches
    assert p.tracker_command(True, -1.0, now=0.1) is False    # armed idle is not flying
    assert p.tracker_command(True, 0.1, now=0.2) is True
    assert p.weld(False, now=5.0) is True and p.attached(now=60.0) is True      # latched
    q = HandoverPolicy()
    q.tracker_command(True, 0.1, now=0.0)                     # warm before the take-back
    assert q.take_back(now=0.1) is True


def test_take_back_with_the_live_rule_off_switches_at_once():
    assert HandoverPolicy(require_live=False).take_back() is True


def test_a_stale_tracker_does_not_get_a_taken_back_drone():
    p = HandoverPolicy(live_timeout_s=0.5)
    p.tracker_command(True, 0.1, now=0.0)
    assert p.take_back(now=2.0) is False


# ── the node ─────────────────────────────────────────────────────────────────────────────────────────────────

pytest.importorskip('interfaces.msg')
import rclpy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402
from interfaces.msg import ELRSCommand  # noqa: E402

from drone_magnet.elrs_mux import ElrsMux  # noqa: E402

DOMAIN = 93


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def _cmd(thr, armed=True):
    return ELRSCommand(armed=armed, channel_0=0.1, channel_1=-0.1, channel_2=thr, channel_3=0.0)


@pytest.fixture
def mux(monkeypatch, request):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    extra = ['-p', f'take_back_topic:={request.param}'] if request.param else []   # unset = the default ''
    rclpy.init(args=['--ros-args', '-p', 'drone_id:=3', *extra], domain_id=DOMAIN)
    n = ElrsMux()
    n.pub = _Rec()
    n.states = n._state_pub = _Rec()
    n.states.msgs.append(String(data=n._state))
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _states(n):
    return [m.data for m in n.states.msgs]


@pytest.mark.parametrize('mux', ['/drone_3/take_back'], indirect=True)
def test_a_take_back_hands_the_drone_to_our_warm_tracker(mux):
    mux._tejen_cb(_cmd(0.2))
    mux._diss_cb(_cmd(0.1))                                   # warm, not forwarded
    assert [m.channel_2 for m in mux.pub.msgs] == [0.2]
    mux._take_back_cb(Bool(data=True))
    mux._diss_cb(_cmd(0.15))
    assert _states(mux) == ['partner', 'ours'] and mux.pub.msgs[-1].channel_2 == 0.15
    k = len(mux.pub.msgs)
    mux._tejen_cb(_cmd(0.3))                                  # his stream is dropped from now on
    mux._attached_cb(Bool(data=False))
    assert len(mux.pub.msgs) == k and _states(mux)[-1] == 'ours'
    assert '/drone_3/take_back' in [s.topic_name for s in mux.subscriptions]


@pytest.mark.parametrize('mux', ['/drone_3/take_back'], indirect=True)
def test_a_false_or_our_silent_tracker_keeps_his_stream(mux):
    mux._take_back_cb(Bool(data=False))
    mux._tejen_cb(_cmd(0.2))
    assert _states(mux) == ['partner']
    mux._take_back_cb(Bool(data=True))
    mux._diss_cb(_cmd(-1.0))                                  # armed idle: not flying
    mux._tejen_cb(_cmd(0.25))
    assert _states(mux) == ['partner'] and mux.pub.msgs[-1].channel_2 == 0.25


@pytest.mark.parametrize('mux', [''], indirect=True)
def test_no_take_back_topic_is_todays_mux(mux):
    assert all('take_back' not in s.topic_name for s in mux.subscriptions)
    mux._tejen_cb(_cmd(0.2))
    mux._diss_cb(_cmd(0.1))
    assert _states(mux) == ['partner'] and [m.channel_2 for m in mux.pub.msgs] == [0.2]
