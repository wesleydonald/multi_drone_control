"""The magnet manager under the tilt cut (card 2026-10-11_quad4_tilt_cut, critic round 1 H1 and H4): the mux's
'cut' turns the magnet OFF even after a confirmed attach (a dead QUAD4 must not hang on plate 9) and refuses every ON
after it; the planner's hand-over block (our drone-3 tracker latched while the partner flies it) turns a proof into
a failed one, the twin joint released. Off ('' topics): today's manager."""
import pytest

pytest.importorskip('interfaces.msg')
from std_msgs.msg import Bool, String  # noqa: E402

from test_magnet_contact_lift_node import _trace, _until, _with_clock, manager  # noqa: E402,F401

CUT = dict(confirm_mode='contact_lift', cut_state_topic='/drone_3/mux_state',
           handover_block_topic='/drone_3/handover_blocked')


def test_a_cut_overrides_the_confirmed_attach_and_refuses_on(manager):
    n = manager(**CUT)
    clock = _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    _until(n, clock, _trace(caught=True), lambda: n.attached)
    n._contact_command_callback(String(data='OFF'))           # refused, as today
    assert n.attached and n.magnet_on and not n.detach_pub.msgs
    n._cut_state_cb(String(data='partner'))                   # not a cut
    assert n.attached
    n._cut_state_cb(String(data='cut'))
    assert not n.attached and not n.magnet_on and len(n.detach_pub.msgs) == 1
    assert n._proof.state == 'off' and n.contact_pub.msgs[-1].data is False
    n._contact_command_callback(String(data='ON'))            # the span's keepalive: refused from now on
    n.magnet_command_callback(String(data='ON'))
    assert not n.magnet_on
    n.timer_callback()
    assert n.attached_pub.msgs[-1].data is False


def test_a_cut_before_any_contact_turns_the_magnet_off_and_keeps_it_off(manager):
    n = manager(**CUT)
    _with_clock(n)
    n._contact_command_callback(String(data='ON'))
    n._cut_state_cb(String(data='cut'))
    assert not n.magnet_on and n._proof.state == 'off'
    n._contact_command_callback(String(data='ON'))
    assert not n.magnet_on and n._proof.state == 'off'


def test_only_a_relaunched_mux_clears_the_cut(manager):
    """Critic round 2, L-B: 'latched' (a fleet DISARM after the cut) keeps it; a fresh 'partner' or 'ours' (only a
    relaunched mux leaves 'cut') clears it, so the drone can be flown again without relaunching the manager."""
    n = manager(**CUT)
    _with_clock(n)
    n._cut_state_cb(String(data='cut'))
    n._cut_state_cb(String(data='latched'))
    n._contact_command_callback(String(data='ON'))
    assert n._cut and not n.magnet_on
    for fresh in ('partner', 'ours'):
        n._cut_state_cb(String(data='cut'))
        n._cut_state_cb(String(data=fresh))
        assert not n._cut
    n._contact_command_callback(String(data='ON'))
    assert n.magnet_on


def test_the_hand_over_block_fails_the_proof_and_releases_the_joint(manager):
    n = manager(**CUT)
    clock = _with_clock(n)
    n._handover_block_cb(Bool(data=True))
    n._contact_command_callback(String(data='ON'))
    trace = _trace(caught=True)
    _until(n, clock, trace, lambda: n.contact_pub.msgs and n.contact_pub.msgs[-1].data)
    _until(n, clock, trace, lambda: n._proof.fails == 1)
    assert not n.attached and all(m.data is False for m in n.attached_pub.msgs)
    assert [m.data for m in n.contact_pub.msgs][-2:] == [True, False]
    assert len(n.attach_pub.msgs) == 1 and len(n.detach_pub.msgs) == 1     # caught, then let go
    assert n._proof.reason == 'handover_blocked' and n._proof.state == 'watch'


def test_off_by_default_the_manager_has_no_cut_inputs(manager):
    n = manager(confirm_mode='contact_lift')
    assert not {'/drone_3/mux_state', '/drone_3/handover_blocked'} & {s.topic_name for s in n.subscriptions}
    assert not n._cut and not n._handover_blocked
