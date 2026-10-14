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


def test_after_the_pickup_his_off_reaches_the_coil_only_at_the_pickup_or_in_the_drop():
    """Critic F9: a LANDING (land_now) or any other OFF of his with the puck on must not drop it over the ring; his
    pickup retry (m1-final) still releases at the pickup site."""
    g = MagnetGate()
    g.his_phase('DESCEND_TO_PICKUP')
    assert g.partner('OFF') == 'OFF'                  # before the puck: harmless, forwarded
    assert g.partner('ON') == 'ON'
    g.puck(True)
    for ph in ('MAGNET_ATTACH_WAIT', 'LIFT_OBJECT', 'APPROACH_ABOVE_PICKUP'):
        g.his_phase(ph)
        assert g.partner('OFF') == 'OFF', ph           # his pickup retry releases at the pickup site
        assert g.partner('ON') == 'ON'
    for ph in ('TRANSIT_TO_DROP_POINT', 'SETTLE_ABOVE_DROP_POINT', 'DESCEND_TO_DROP_HEIGHT', 'LANDING',
               'TAKEN_BACK', ''):
        g.his_phase(ph)
        assert g.partner('OFF') is None, ph            # held: the puck stays on
    assert g.last == 'ON'
    g.his_phase('DROP_OBJECT')
    assert g.partner('OFF') == 'OFF'                  # the drop
    g.puck(False)
    for ph in ('TRANSIT_TO_REATTACH', 'TAKEN_BACK'):  # after the drop his OFFs pass (d8c1b65: TAKEN_BACK holds OFF)
        g.his_phase(ph)
        assert g.partner('OFF') == 'OFF', ph
    g.puck(True)                                      # the puck re-caught after the drop (R1520): OFF still passes
    assert g.partner('OFF') == 'OFF'


def test_our_on_takes_the_coil_only_with_the_puck_released():
    """Critic F9: an ON before the drop (RViz ATTACH, or a span begun early) must not take the coil with the puck."""
    g = MagnetGate()
    g.partner('ON')
    g.puck(True)
    assert g.ours('ON') is None and g.source == 'partner'
    g.his_phase('DROP_OBJECT')
    assert g.partner('OFF') == 'OFF'
    g.puck(False)
    assert g.ours('ON') == 'ON' and g.source == 'ours'
    assert sum('our ON held' in e for e in g.events) == 1


def test_full_m1_welded_start_ours_then_his_after_the_release_then_ours_at_our_span():
    g = MagnetGate(start_ours=True)
    assert g.source == 'ours' and g.ours('ON') == 'ON' and g.partner('OFF') is None   # welded: ours
    assert g.ours('OFF') == 'OFF'                     # our detach releases QUAD4 from the ring
    g.release()
    assert g.source == 'partner' and g.partner('ON') == 'ON'      # his pickup
    g.puck(True)
    g.his_phase('DROP_OBJECT')
    assert g.partner('OFF') == 'OFF'
    g.puck(False)
    assert g.ours('ON') == 'ON' and g.source == 'ours'
    g.release()                                       # once only
    assert g.source == 'ours'
    assert MagnetGate().source == 'partner'
    h = MagnetGate()
    h.release()                                       # no welded start: a release changes nothing
    assert h.source == 'partner'


def test_restore_takes_ours_back_and_ignores_partner():
    g = MagnetGate()
    g.restore('partner')
    assert g.source == 'partner'
    g.restore('ours')
    assert g.source == 'ours' and g.partner('OFF') is None


@pytest.mark.parametrize('start_ours,span', [(False, False), (False, True), (True, False)])
def test_the_cut_turns_the_coil_off_and_drops_both_inputs_for_good(start_ours, span):
    """The tilt cut (card 2026-10-11_quad4_tilt_cut, critic round 2 H-B): his stream flies QUAD4 (the gate his, or
    ours once our span began in his ring descent); the mux's 'cut' turns the coil OFF and nothing turns it on again:
    not his rejoin's ON, not ours, not a release or a restore."""
    g = MagnetGate(start_ours=start_ours)
    g.partner('ON')
    if span:
        g.ours('ON')                                  # our span began (his descent onto plate 9)
    assert g.cut() == 'OFF' and g.source == 'cut' and g.last == 'OFF'
    assert g.partner('ON') is None and g.ours('ON') is None and g.ours('OFF') is None
    g.release()
    g.restore('ours')
    assert g.source == 'cut' and g.partner('ON') is None
    assert sum('ignored: the coil is cut' in e for e in g.events) == 1 and g.cut() == 'OFF'


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


def _node(monkeypatch, *extra):
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=['--ros-args', '-p', 'drone_id:=3', *extra], domain_id=DOMAIN)
    from drone_magnet.magnet_gate import MagnetGateNode
    n = MagnetGateNode()
    n.pub, n.state_pub = _Rec(), _Rec()
    return n


def test_node_forwards_and_reports_its_source(monkeypatch):
    try:
        n = _node(monkeypatch, '-p', 'restore_s:=0.0')
        n._on(n.gate.ours, 'OFF')
        n._on(n.gate.partner, 'ON')
        n._on(n.gate.ours, 'ON')
        n._on(n.gate.partner, 'OFF')
        assert [m.data for m in n.pub.msgs] == ['ON', 'ON']
        assert [m.data for m in n.state_pub.msgs] == ['ours']
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_node_restart_restores_ours_and_forwards_nothing_meanwhile(monkeypatch):
    """A restarted gate (low, critic r1): during restore_s it moves its state but writes nothing; a previous
    instance's latched 'ours' is taken back at once, so his commands never reach the coil again."""
    try:
        n = _node(monkeypatch, '-p', 'restore_s:=5.0')
        n._on(n.gate.partner, 'OFF')                    # his, during the window: not forwarded
        n._state_cb(String(data='ours'))                # the previous instance's latched state
        n._on(n.gate.partner, 'ON')
        n._on(n.gate.ours, 'ON')                         # our keepalive
        assert n.pub.msgs == [] and n.gate.source == 'ours'
        n._end_restore()
        assert [m.data for m in n.pub.msgs] == ['ON'] and [m.data for m in n.state_pub.msgs] == ['ours']
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_node_cut_publishes_off_at_once_even_while_restoring_and_resends_it(monkeypatch):
    """H-B: the mux's latched 'cut' (cut_state_topic) reaches the coil within the callback, restoring or not; the
    state reads 'cut'; nothing else reaches the coil; 'partner' / 'ours' / 'latched' do not cut. Off by default."""
    try:
        n = _node(monkeypatch, '-p', 'restore_s:=5.0', '-p', 'cut_state_topic:=/drone_3/mux_state')
        assert '/drone_3/mux_state' in {x.topic_name for x in n.subscriptions}
        for st in ('partner', 'ours', 'latched'):
            n._mux_state_cb(String(data=st))
        assert n.pub.msgs == [] and n.gate.source == 'partner'
        n._mux_state_cb(String(data='cut'))
        assert [m.data for m in n.pub.msgs] == ['OFF'] and [m.data for m in n.state_pub.msgs] == ['cut']
        n._mux_state_cb(String(data='cut'))           # the latched state again: no second cut
        n._end_restore()
        n._on(n.gate.partner, 'ON')
        n._on(n.gate.ours, 'ON')
        assert [m.data for m in n.pub.msgs] == ['OFF'] and [m.data for m in n.state_pub.msgs] == ['cut']
        n._cut_timer.callback()                       # the 2 Hz re-send
        assert [m.data for m in n.pub.msgs] == ['OFF', 'OFF']
        n.destroy_node()
    finally:
        rclpy.shutdown()
    try:
        n = _node(monkeypatch, '-p', 'restore_s:=0.0')
        assert '/drone_3/mux_state' not in {x.topic_name for x in n.subscriptions}
        n.destroy_node()
    finally:
        rclpy.shutdown()
