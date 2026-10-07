"""Drop and land (card 2026-10-08_drop_and_land): the planner's latched /fleet/payload_dropped puts
the manager into landing, so the planner's /fleet/landed disarms the fleet as after a LAND; not
flying, it changes nothing."""
from std_msgs.msg import Bool

import fleet_manager.fleet_manager_node as m
from test_mux_abort_manager import _fake as _base_fake, sync  # noqa: F401
from test_pre_takeoff_gate import _armed


def _fake():
    f = _base_fake()
    for name in ('_payload_dropped_callback', '_landed_callback'):
        setattr(f, name, getattr(m.CentralController, name).__get__(f))
    return f


def test_dropped_while_flying_lands_then_disarms_on_landed(sync):
    f = _fake()
    _armed(f)
    f._takeoff_fleet()
    assert f.flying and not f.landing
    f._payload_dropped_callback(Bool(data=True))
    assert f.landing and any('PAYLOAD DROPPED' in s for s in f._log.lines)
    f._landed_callback(Bool(data=True))
    assert not f.flying and f.service_disarms


def test_dropped_false_or_not_flying_does_nothing(sync):
    f = _fake()
    f._payload_dropped_callback(Bool(data=True))
    assert not f.landing
    _armed(f)
    f._takeoff_fleet()
    f._payload_dropped_callback(Bool(data=False))
    assert not f.landing
