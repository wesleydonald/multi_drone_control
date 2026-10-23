"""land_rearm_touchdown's stale-pose rule (card 2026-10-17_touchdown_height_gate, critic on the rig path): a pose older
than LAND_POSE_FRESH_S holds the stall count at zero, a fresh one lets it count, and a welded QUAD4 folded into a slot
must be stamped like a carrier (it was not: the fleet then never saw a stall and landed by max drop)."""
import time
import types

import numpy as np

from mpc_planner.planner_node import LAND_POSE_FRESH_S, LoadPlanner, TouchdownDetector

HZ, LAND_VEL = 10.0, 0.2


def _fake(stamps, n=3):
    f = types.SimpleNamespace(n=n, slot2drone=list(range(n)), _land_rearm=True, _drone_t=stamps,
                              _touchdown=TouchdownDetector(LAND_VEL, HZ), z=0.40)
    f._drone_at = lambda i: np.array([0.0, 0.0, f.z])
    return f


def _stalled_after(f, ticks):
    """Descend at land_vel to 0.10 m, then sit still: the stall test's own picture of a landing."""
    fired = False
    for _ in range(ticks):
        f.z = max(0.10, f.z - LAND_VEL / HZ)
        fired = LoadPlanner._touchdown_stalled(f) or fired
    return fired


def test_fresh_poses_reach_the_stall():
    now = time.monotonic()
    f = _fake({0: now, 1: now, 2: now})
    assert _stalled_after(f, int(6 * HZ))           # 1.5 s down, then the 1.0 s stall


def test_one_stale_pose_holds_the_count():
    now = time.monotonic()
    f = _fake({0: now, 1: now, 2: now - 10 * LAND_POSE_FRESH_S})
    assert not _stalled_after(f, int(6 * HZ))
    assert f._touchdown._stall_ct == 0


def test_unstamped_slot_reads_stale():
    """The welded-QUAD4 case before the fix: a slot with no stamp never lets the stall count."""
    now = time.monotonic()
    f = _fake({0: now, 1: now, 2: now}, n=4)
    assert not _stalled_after(f, int(6 * HZ))


def test_welded_quad4_is_stamped():
    """dissipative_node._attach_drone_cb stamps _drone_t for an OCP-attached newcomer."""
    from dissipative_planner.dissipative_node import DissipativeController
    msg = types.SimpleNamespace(pose=types.SimpleNamespace(position=types.SimpleNamespace(x=0.1, y=0.2, z=0.3),
                                                           orientation=types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)),
                                twist=types.SimpleNamespace(linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0)))
    clock = types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=0))
    f = types.SimpleNamespace(attach_pos={}, _attach_vel={}, _attach_pos_t={}, _n_carry0=3, _ocp_attached={3: True},
                              drone_pos=[None] * 4, drone_vel=[None] * 4, drone_quat=[None] * 4, _drone_t={},
                              get_clock=lambda: clock)
    t0 = time.monotonic()
    DissipativeController._attach_drone_cb(f, msg, 0)
    assert f._drone_t[3] >= t0 and np.allclose(f.drone_pos[3], [0.1, 0.2, 0.3])
