"""carriers_down_alone (card 2026-11-05_carriers_down_alone, critic CODE 1): the carriers stay armed while a take-back
can still pull the landed ring: in its hold, its release guard or guard_follow, or after its guard failed (the tip may
still be on plate 9; the step out by GUARD_OUT_M runs once the ring is down) until its own reference ends."""
from types import SimpleNamespace

import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402


def _busy(comove=None):
    return D._take_back_busy(SimpleNamespace(_comove=comove or {}))


def test_no_take_back_or_one_past_its_guard_is_not_busy():
    assert _busy() is None
    assert _busy({3: {'stage': 'move', 'guard': None}}) is None
    assert _busy({3: {'stage': 'land', 'guard': {'fail_t': None}}}) is None      # a guard that passed


@pytest.mark.parametrize('stage', ['hold', 'guard', 'guard_follow'])
def test_hold_guard_and_guard_follow_are_busy(stage):
    why = _busy({3: {'stage': stage, 'guard': None}})
    assert why is not None and stage in why


def test_a_failed_guard_is_busy_until_its_reference_ends():
    why = _busy({3: {'stage': 'land', 'guard': {'fail_t': 12.0}}})
    assert why is not None and 'failed' in why
