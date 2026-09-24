"""ZBias: integrates the height miss only while gated, is bounded, frozen (not zeroed)
when the gate drops, reset on demand, and flags the near-bound state (fault masking)."""
from controller_load_mpc.planner_node import ZBias

HZ = 10.0


def _run(zb, err, seconds, gated=True):
    for _ in range(int(seconds * HZ)):
        zb.update(err, gated)
    return zb.value


def test_off_by_default_does_nothing():
    zb = ZBias(0.0, 0.15, HZ)
    assert _run(zb, -0.06, 20.0) == 0.0 and zb.n_updates == 0


def test_integrates_the_miss_at_tau_one_over_ki():
    zb = ZBias(0.2, 0.15, HZ)
    v = _run(zb, -0.06, 5.0)                 # 1 tau of a constant -6 cm miss
    assert abs(v - (-0.06)) < 1e-9           # ki*err*t = 0.2*-0.06*5


def test_bounded_and_flags_near_bound():
    zb = ZBias(0.2, 0.15, HZ)
    assert not zb.near_bound
    _run(zb, -0.30, 30.0)
    assert zb.value == -0.15 and zb.near_bound


def test_frozen_not_zeroed_when_the_gate_drops():
    zb = ZBias(0.2, 0.15, HZ)
    _run(zb, -0.06, 5.0)
    held = zb.value
    _run(zb, +0.50, 10.0, gated=False)
    assert zb.value == held


def test_reset():
    zb = ZBias(0.2, 0.15, HZ)
    _run(zb, -0.06, 5.0); zb.reset()
    assert zb.value == 0.0 and zb.n_updates == 0 and not zb.near_bound
