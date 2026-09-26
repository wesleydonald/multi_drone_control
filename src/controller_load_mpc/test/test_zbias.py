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


def _gate_stub(traj_kind, traj_t, in_orbit, speed=0.125, advancing=True):
    import time
    from types import SimpleNamespace
    import numpy as np
    ls = np.zeros(13); ls[2] = 0.65
    from types import MethodType
    from controller_load_mpc.planner_node import LoadPlanner
    st = SimpleNamespace(
        lift_z0=0.0, load_state=ls, _load_t=time.monotonic(), lift_progress=0.6, target_z=0.6,
        descending=False, _land_to_ground=False, traj_t=traj_t,
        traj=SimpleNamespace(kind=traj_kind, speed=speed, radius=0.5), _z_ki_in_orbit=in_orbit, n=0,
        _zbias_traj_t_prev=traj_t - 0.1 if advancing else traj_t,
        _drone_at=lambda i: None, _cable_taut_gate=lambda i: (1.0,), _z_taut_gate=0.99)
    st._zbias_orbit_ok = MethodType(LoadPlanner._zbias_orbit_ok, st)
    return st


def test_gate_frozen_in_a_trajectory_unless_orbit_opt_in():
    from controller_load_mpc.planner_node import LoadPlanner
    g = LoadPlanner._zbias_gated
    assert g(_gate_stub('hover', 0.0, False))
    assert not g(_gate_stub('orbit', 30.0, False))
    assert g(_gate_stub('orbit', 30.0, True))
    assert not g(_gate_stub('orbit', 2.0, True))       # still spinning up
    assert not g(_gate_stub('circle', 30.0, True))
    assert not g(_gate_stub('orbit', 30.0, True, advancing=False))   # frozen clock: a hold
    assert not g(_gate_stub('orbit', 30.0, True, speed=0.6))          # 0.72 m/s^2 sags

