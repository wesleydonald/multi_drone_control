"""The cable feedforward gate on a resting ring follows the planner's latched
/fleet/cable_ff_active flag: off through creep and settle, on for the whole pretension ramp
on every drone at once (rig 2026-09-30: blocking it deadlocked the lift; opening it late
stepped the full pull on and launched the ring)."""
import types

from tracker.tracker_node import Controller


def _g(resting, planner_pulling):
    s = types.SimpleNamespace(payload_resting=resting, _planner_ff_active=planner_pulling)
    return Controller._load_grounded(s)


def test_resting_without_the_planner_pulling_is_grounded():
    assert _g(True, False)


def test_planner_pulling_opens_the_gate_on_the_floor():
    assert not _g(True, True)


def test_airborne_is_never_grounded():
    assert not _g(False, False)
    assert not _g(False, True)
