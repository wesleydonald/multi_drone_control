"""The one writer of QUAD4's coil in the combined M1 contact attach (combined-stack O5; card
2026-10-10_combined_m1_contact, critic round 3 finding 6): his ball commands until our span, ours after it.
Falsifiers: an OFF of ours reaches the coil before our first ON (his ball dropped by a take-back), or an OFF
of his reaches it after (QUAD4 dropped off the ring)."""
import pytest

from drone_magnet.magnet_gate import MagnetGate, magnet_word


def test_words():
    assert [magnet_word(w) for w in ('on', ' ON ', 'true', 'off', 'OFF', 'NONE', '', 'x')] == [
        'ON', 'ON', 'ON', 'OFF', 'OFF', None, None, None]


def test_his_ball_cycle_passes_and_our_off_keepalive_never_does():
    g = MagnetGate()
    assert g.ours('OFF') is None                      # the manager's boot keepalive
    assert g.partner('OFF') == 'OFF'                  # his pickup approach
    assert g.partner('ON') == 'ON'                    # his pickup descent: the ball
    for _ in range(5):
        assert g.ours('OFF') is None                  # a take-back before our span: the ball stays on
    assert g.partner('ON') == 'ON' and g.source == 'partner'
    assert g.partner('OFF') == 'OFF'                  # his drop over the net
    assert g.partner('NONE') is None                  # his TAKEN_BACK / RING_ATTACHED send nothing


def test_our_first_on_takes_the_coil_for_good():
    g = MagnetGate()
    g.partner('ON')
    g.partner('OFF')                                  # the drop
    assert g.ours('ON') == 'ON' and g.source == 'ours'
    assert g.partner('OFF') is None                   # his TRANSIT/APPROACH/MATCH phases send OFF
    assert g.partner('ON') is None
    assert g.ours('ON') == 'ON'                       # our keepalive
    assert g.ours('OFF') == 'OFF'                     # our take-back after the span: release
    assert g.ours('ON') == 'ON' and g.source == 'ours'
    assert g.partner('ON') is None                    # nothing hands it back


def test_events_name_the_switch_and_each_first_drop_once():
    g = MagnetGate()
    g.ours('OFF')
    g.ours('OFF')
    g.partner('ON')
    g.ours('ON')
    g.partner('OFF')
    g.partner('OFF')
    text = '\n'.join(g.events)
    assert text.count('our OFF ignored before our span') == 1
    assert text.count('OURS from now') == 1
    assert text.count("his OFF ignored") == 1
    assert 'coil ON (his command)' in text


@pytest.mark.parametrize('seq,coil', [
    # (input, word) sequences and the last word on the coil
    ([('p', 'ON'), ('o', 'OFF'), ('o', 'OFF')], 'ON'),                 # take-back before the span
    ([('p', 'ON'), ('p', 'OFF'), ('o', 'ON'), ('p', 'OFF')], 'ON'),    # span, then his rejoin OFFs
    ([('p', 'ON'), ('p', 'OFF'), ('o', 'ON'), ('o', 'OFF')], 'OFF'),   # take-back after the span
])
def test_sequences(seq, coil):
    g = MagnetGate()
    last = None
    for who, w in seq:
        out = g.partner(w) if who == 'p' else g.ours(w)
        last = out if out is not None else last
    assert last == coil


# ── the node: one writer of /drone_3/magnet ─────────────────────────────────────────────

pytest.importorskip('rclpy')
import rclpy  # noqa: E402
from std_msgs.msg import String  # noqa: E402

DOMAIN = 94


class _Rec:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


def test_node_forwards_and_reports_its_source(monkeypatch):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=['--ros-args', '-p', 'drone_id:=3'], domain_id=DOMAIN)
    try:
        from drone_magnet.magnet_gate import MagnetGateNode
        n = MagnetGateNode()
        n.pub, n.state_pub = _Rec(), _Rec()
        n._on(n.gate.ours, String(data='OFF'))
        n._on(n.gate.partner, String(data='ON'))
        n._on(n.gate.ours, String(data='ON'))
        n._on(n.gate.partner, String(data='OFF'))
        assert [m.data for m in n.pub.msgs] == ['ON', 'ON']
        assert [m.data for m in n.state_pub.msgs] == ['ours']
        n.destroy_node()
    finally:
        rclpy.shutdown()
