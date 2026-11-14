"""The newcomer's tracker while another stream flies it (RIG-1105-k1ob_2): while its mux reads 'partner' the ring's tilt
neither faults it nor asks for a drop; 'ours', 'cut', 'latched' or nothing heard yet: the checks apply as before."""
from types import MethodType, SimpleNamespace

from tracker.tracker_node import Controller as N  # noqa: N814


def _stub(off_ring):
    logs = []
    st = SimpleNamespace(_off_ring=off_ring, _payload_dropped=False, _drop_requested_t=None, drone_id=3,
                         drop_on_loss=True, payload_rest_z=0.0275, get_logger=lambda: SimpleNamespace(warn=logs.append))
    st._payload_checks_down = MethodType(N._payload_checks_down, st)
    st._off_ring_cb = MethodType(N._off_ring_cb, st)
    return st, logs


def test_flown_by_the_other_stream_the_ring_checks_stand_down():
    assert _stub(True)[0]._payload_checks_down() is True


def test_ours_or_unknown_the_checks_apply():
    assert _stub(False)[0]._payload_checks_down() is False


def test_the_flag_follows_the_mux_and_says_so_once_per_change():
    st, logs = _stub(False)
    st._off_ring_cb(SimpleNamespace(data='partner'))
    assert st._off_ring is True and len(logs) == 1 and 'stand down' in logs[0]
    st._off_ring_cb(SimpleNamespace(data='partner'))
    assert len(logs) == 1
    st._off_ring_cb(SimpleNamespace(data='ours'))
    assert st._off_ring is False and 'apply' in logs[-1]
    for state in ('cut', 'latched'):
        st._off_ring_cb(SimpleNamespace(data='partner'))
        st._off_ring_cb(SimpleNamespace(data=state))
        assert st._off_ring is False


def test_off_the_ring_no_drop_rule_runs():
    st, _ = _stub(True)
    called = []
    st._drop_rule = lambda q: called.append(q)
    msg = SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0, z=1.0),
                                               orientation=SimpleNamespace(w=0.5, x=0.5, y=0.5, z=0.5)),
                          twist=SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0)))
    N._payload_state_cb(st, msg)
    assert called == []
    st._off_ring = False
    N._payload_state_cb(st, msg)
    assert len(called) == 1


def test_the_existing_handover_handler_is_not_shadowed():
    assert N._off_ring_cb is not N._mux_state_cb
