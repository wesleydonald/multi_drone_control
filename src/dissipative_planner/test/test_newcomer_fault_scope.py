"""A fault on an unwelded newcomer lands the fleet (card 2026-10-09_newcomer_fault_scope; Wesley 8 Oct night: "if
QUAD4 fails we just land everything"): the planner keeps each reserved drone's latched on_load (False until it is
folded in, False again at its detach) for the fleet manager, and a free newcomer that disarms in flight is out of
the flight (magnet OFF, no landed-gate wait, never folded in)."""
from types import MethodType, SimpleNamespace

import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402

from test_newcomer_lifecycle import _node as _lifecycle_node  # noqa: E402


def _node(**kw):
    f = _lifecycle_node(**kw)
    f._newcomer_fault_land = True
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


def test_a_refused_physical_weld_does_not_put_it_on_the_ring_for_the_manager():
    g = _node()
    g._abort_newcomer(0, 'abort')
    g._magnet_attached_cb(Bool(data=True))                         # caught on its way out: refused
    assert True not in g._on_load and g.attached == []
    g._magnet_attached_cb(Bool(data=False))                        # the OFF let go
    assert g._on_load == [False]


def test_a_disarmed_newcomer_is_never_folded_in_even_before_its_fault_mark():
    """The weld-order race: the weld arrives after the disarm but before the edge is handled."""
    f = _node()
    f._newcomer_armed[0] = False                                   # its last feedback: disarmed
    f._magnet_attached_cb(Bool(data=True))
    assert f.attached == [] and any('it disarmed in flight' in m for m in f.log.lines)
    g = _node()
    g._newcomer_armed[0] = True
    g._magnet_attached_cb(Bool(data=True))
    assert g.attached == [3]


def test_a_fault_clears_a_held_trajectory_as_an_abort_does():
    f = _node()
    f._attach_moving, f._reconfig_hold_left = False, 60.0          # the approach's trajectory hold
    _edge(f)
    assert f._reconfig_hold_left == 0.0 and not f._approach_hold_armed


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

