"""TouchdownDetector: touchdown is the stall of the drones being landed, armed only
after a real descent, and never held open by a drone that is not in the set (the
R0113/R0114 LAND abort: a detached drone still descending at land_vel kept the max
moving while the survivors sat on the floor)."""
from controller_load_mpc.planner_node import (
    TouchdownDetector, LAND_GRACE_S, LAND_STALL_S, LAND_MIN_DESCENT)

HZ, LAND_VEL = 10.0, 0.2
DT = 1.0 / HZ


def _descend(det, zs0, floors, seconds, extra=None):
    """Feed `seconds` of ticks: each drone descends at land_vel until it reaches its
    floor. `extra` is an optional list of (z0, floor) NOT in the set unless
    `include_extra`. Returns the first time the detector fired, or None."""
    zs = list(zs0)
    fired = None
    for k in range(int(seconds * HZ)):
        zs = [max(f, z - LAND_VEL * DT) for z, f in zip(zs, floors)]
        if det.update(zs) and fired is None:
            fired = (k + 1) * DT
    return fired


def test_fires_once_all_landed_drones_stall():
    det = TouchdownDetector(LAND_VEL, HZ)
    t = _descend(det, [0.40, 0.40, 0.40], [0.10, 0.10, 0.10], 10.0)
    # 0.30 m at 0.2 m/s = 1.5 s to the floor, then LAND_STALL_S of stall
    assert t is not None and 1.5 + LAND_STALL_S - DT <= t <= 1.5 + LAND_STALL_S + 3 * DT


def test_a_departed_drone_outside_the_set_does_not_hold_it_open():
    """Survivors fed alone fire; the same survivors plus a drone still descending
    from 0.66 m (R0113's parked drone) do not fire while it moves."""
    alone = TouchdownDetector(LAND_VEL, HZ)
    t_alone = _descend(alone, [0.40, 0.40, 0.40], [0.10, 0.10, 0.10], 6.0)
    assert t_alone is not None
    with_departed = TouchdownDetector(LAND_VEL, HZ)
    t_all = _descend(with_departed, [0.40, 0.40, 0.40, 0.66], [0.10, 0.10, 0.10, 0.0], 3.5)
    assert t_all is None            # the fourth is still descending at 3.5 s (0.66/0.2 = 3.3 s + stall)


def test_no_fire_before_a_real_descent():
    """Stationary drones (a lagging tracker) at altitude are not 'landed'."""
    det = TouchdownDetector(LAND_VEL, HZ)
    for _ in range(int((LAND_GRACE_S + LAND_STALL_S + 2.0) * HZ)):
        assert not det.update([0.90, 0.90, 0.90])


def test_arms_only_after_min_descent():
    det = TouchdownDetector(LAND_VEL, HZ)
    # descend less than LAND_MIN_DESCENT then stop: no fire
    z = 0.90
    for k in range(int(6.0 * HZ)):
        z = max(0.90 - LAND_MIN_DESCENT + 0.01, z - LAND_VEL * DT)
        assert not det.update([z, z, z])


def test_reset_clears_state_for_a_repeat_land():
    det = TouchdownDetector(LAND_VEL, HZ)
    assert _descend(det, [0.40, 0.40], [0.10, 0.10], 6.0) is not None
    det.reset()
    for _ in range(int(LAND_GRACE_S * HZ) - 1):
        assert not det.update([0.10, 0.10])


def test_missing_height_is_not_touchdown():
    det = TouchdownDetector(LAND_VEL, HZ)
    assert not det.update([]) and not det.update([0.1, None])


def test_replay_r0113_survivors_fire_before_the_recorded_fault():
    """Replay R0113's recorded survivor heights (detach_ocp_n4, LAND at 92.003 s, tilt
    fault at 95.365 s) through the detector at the planner rate: the survivors-only
    stall must fire before the fault, and a departed drone descending at 2x land_vel
    from its 0.66 m hold is down before that. (The fault itself came from the cable
    feedforward pushing a grounded drone sideways; that is closed by the planner's
    feedforward-off-when-the-load-is-down rule, not by this timing.)"""
    import csv, os
    import numpy as np
    import pytest
    f = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'results', '2026-08-06',
                     'R0113_sim_gz_detach_ocp_n4', 'logs', 'run.csv')
    if not os.path.exists(f):
        pytest.skip('R0113 log not present')
    rows = list(csv.DictReader(open(f)))
    t = np.array([float(r['t']) for r in rows])
    zs = {k: np.array([float(r[f'd{k}_z']) for r in rows]) for k in range(4)}
    hz, land_vel, t_land, t_fault = 10.0, 0.2, 92.003, 95.365
    det = TouchdownDetector(land_vel, hz)
    dep = TouchdownDetector(2 * land_vel, hz)
    fired = dep_down = None
    z3 = 0.66
    for k in range(int((t_fault - t_land) * hz)):
        tk = t_land + (k + 1) / hz
        surv = [float(np.interp(tk, t, zs[i])) for i in range(3)]
        if det.update(surv) and fired is None:
            fired = tk
        z3 = max(0.0, z3 - 2 * land_vel / hz)         # synthetic 2x descent of the freed drone
        if dep_down is None and (z3 <= 0.15 or dep.update([z3 + 0.10])):
            dep_down = tk
    assert fired is not None and fired < t_fault - 0.3, fired      # ~95.0 s vs 95.365
    assert dep_down is not None and dep_down < fired, (dep_down, fired)
