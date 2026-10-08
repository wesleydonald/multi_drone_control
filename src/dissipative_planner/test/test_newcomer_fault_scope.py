"""A fault on an unwelded newcomer grounds it alone (card 2026-10-09_newcomer_fault_scope; Wesley 8 Oct night): the
planner keeps each reserved drone's latched on_load (False until its weld, True while it is physically on the
ring, False again at its detach) for the fleet manager, and a free newcomer that disarms in flight is out of the
flight (magnet OFF, no landed-gate wait, no later weld)."""
from types import MethodType, SimpleNamespace

import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from test_newcomer_lifecycle import _node as _lifecycle_node  # noqa: E402


def _node(**kw):
    f = _lifecycle_node(**kw)
    f._newcomer_fault_alone = True
    f._newcomer_armed = {}
    f._on_load = []
    f._on_load_pubs = {3: SimpleNamespace(publish=lambda m: f._on_load.append(m.data))}
    f._newcomer_arming_cb = MethodType(D._newcomer_arming_cb, f)
    return f


def _edge(f):
    f._newcomer_arming_cb(Bool(data=True), 0)
    f._newcomer_arming_cb(Bool(data=False), 0)


def test_a_free_newcomer_that_disarms_in_flight_is_out_and_not_waited_for():
    f = _node()
    f._abort_newcomer(0, 'abort')                                  # on its way out when it faults
    _edge(f)
    assert f._newcomer_out == {0: 'fault'}
    assert 3 not in f._departed_hold and 3 not in f._land_alone and 3 not in f._departed_wait
    assert f._departed_down()                                      # nothing left to wait for


def test_a_fault_during_the_approach_sends_off_and_blocks_any_weld():
    f = _node()
    _edge(f)
    assert f._newcomer_out == {0: 'fault'} and 0 not in f._approach and f._magnet_cmd_pub.msgs == ['OFF']
    f._magnet_attached_cb(Bool(data=True))
    assert f.attached == [] and any('it disarmed in flight' in m for m in f.log.lines)
    f._newcomer_lifecycle = False                                  # refused with either switch
    f._magnet_attached_cb(Bool(data=True))
    assert f.attached == []


@pytest.mark.parametrize('kw', [{'takeoff_seen': False}, {'_landed': True}, {'pending': False}])
def test_disarm_edges_that_are_not_a_free_newcomer_fault(kw):
    pending = kw.pop('pending', True)
    f = _node(pending=pending)
    for k, v in kw.items():
        setattr(f, k, v)
    _edge(f)
    assert f._newcomer_out == {} and f._magnet_cmd_pub.msgs == []


def test_on_load_follows_the_physical_weld_and_a_refusal():
    f = _node()
    f._magnet_attached_cb(Bool(data=True))
    assert f._on_load == [True] and f.attached == [3]
    g = _node()
    g._abort_newcomer(0, 'abort')
    g._magnet_attached_cb(Bool(data=True))                         # welded on its way out: refused
    assert g._on_load == [True] and g.attached == []
    g._magnet_attached_cb(Bool(data=False))                        # the OFF let go
    assert g._on_load == [True, False]


def test_arm_before_takeoff_clears_a_fault_from_a_refused_arm():
    f = _node()
    _edge(f)
    f.takeoff_seen = False
    f._newcomers_out_on_land('ARM')
    assert f._newcomer_out == {}
    f._newcomer_lifecycle = False
    f._newcomer_out[0] = 'fault'
    f._newcomers_out_on_land('ARM')                                # whichever switch is on
    assert f._newcomer_out == {}

