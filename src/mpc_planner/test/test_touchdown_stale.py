"""land_rearm_touchdown's stale-pose rule (card 2026-10-17_touchdown_height_gate, critic on the rig path): a pose older
than LAND_POSE_FRESH_S holds the stall count at zero, a fresh one lets it count, and a welded QUAD4 folded into a slot
must be stamped like a carrier (it was not: the fleet then never saw a stall and landed by max drop)."""
import time
import types

import numpy as np

from mpc_planner.planner_node import LAND_POSE_FRESH_S, LoadPlanner, TouchdownDetector

HZ, LAND_VEL = 10.0, 0.2


def _fake(stamps, n=3, rearm=True):
    lines = []
    log = types.SimpleNamespace(warn=lambda m: lines.append(('warn', m)), info=lambda m: lines.append(('info', m)))
    f = types.SimpleNamespace(n=n, slot2drone=list(range(n)), _land_rearm=rearm, _drone_t=stamps,
                              _touchdown=TouchdownDetector(LAND_VEL, HZ), z=0.40, _land_stale={}, _land_stale_run={},
                              get_logger=lambda: log, lines=lines)
    f._drone_at = lambda i: np.array([0.0, 0.0, f.z])
    f._note_stale_poses = lambda stale: LoadPlanner._note_stale_poses(f, stale)
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
                              drone_pos=[None] * 4, drone_vel=[None] * 4, drone_quat=[None] * 4, _drone_t={}, _drone_stamp={},
                              get_clock=lambda: clock)
    t0 = time.monotonic()
    DissipativeController._attach_drone_cb(f, msg, 0)
    assert f._drone_t[3] >= t0 and np.allclose(f.drone_pos[3], [0.1, 0.2, 0.3])


# ── the rig-path critic's should-fixes (17 Oct; the rig default since 5 Nov): logged only, the stall test unchanged ──

def _touchdown_stalled_reviewed(self):
    """_touchdown_stalled's re-arm branch as reviewed on 17-18 Oct (week8 9f20f8b), frozen here: the counter and the
    dropout log must not change which tick fires."""
    zs = [float(self._drone_at(i)[2]) for i in range(self.n)]
    now = time.monotonic()
    if any(now - self._drone_t.get(d, -1e9) > LAND_POSE_FRESH_S for d in self.slot2drone[:self.n]):
        self._touchdown._stall_ct = 0
        return False
    return self._touchdown.update(zs)


def _dropouts(f, plan):
    """Run the stall test with drone 2's pose stale on the ticks `plan` marks True; return each tick's answer."""
    out = []
    for stale in plan:
        now = time.monotonic()
        f._drone_t.update({0: now, 1: now, 2: now - (10 * LAND_POSE_FRESH_S if stale else 0.0)})
        f.z = max(0.10, f.z - LAND_VEL / HZ)
        out.append((LoadPlanner._touchdown_stalled(f), f._touchdown._stall_ct))
    return out


PLAN = [False] * 20 + [True] * 4 + [False] * 6 + [True] * 3 + [False] * 30


def test_the_stale_count_changes_no_tick():
    a, b = _fake({}), _fake({})
    ref = []
    for stale in PLAN:
        now = time.monotonic()
        b._drone_t.update({0: now, 1: now, 2: now - (10 * LAND_POSE_FRESH_S if stale else 0.0)})
        b.z = max(0.10, b.z - LAND_VEL / HZ)
        ref.append((_touchdown_stalled_reviewed(b), b._touchdown._stall_ct))
    assert _dropouts(a, PLAN) == ref and any(fired for fired, _ in ref)


def test_stale_ticks_are_counted_per_drone_for_the_touchdown_line():
    f = _fake({})
    _dropouts(f, PLAN)
    assert f._land_stale == {2: 7} and f._land_stale_run == {}
    assert LoadPlanner._land_stale_text(f) == '; stale-pose ticks 7 (d2 7)'
    g = _fake({})
    _dropouts(g, [False] * 10)
    assert LoadPlanner._land_stale_text(g) == '; stale-pose ticks 0'
    assert LoadPlanner._land_stale_text(_fake({}, rearm=False)) == ''      # the line as before with the re-arm off


def test_each_dropout_logs_once_as_it_starts_and_once_as_it_ends():
    f = _fake({})
    _dropouts(f, PLAN)
    warns = [m for k, m in f.lines if k == 'warn']
    infos = [m for k, m in f.lines if k == 'info']
    assert len(warns) == 2 and all('d2 pose' in m and 'Motive dropout?' in m for m in warns)
    assert infos == ['[planner] LAND stall test: d2 pose fresh again after 4 stale tick(s) (0.4 s)',
                     '[planner] LAND stall test: d2 pose fresh again after 3 stale tick(s) (0.3 s)']
    h = _fake({0: time.monotonic()}, n=2)                  # a slot never stamped
    LoadPlanner._touchdown_stalled(h)
    LoadPlanner._touchdown_stalled(h)
    assert [m for k, m in h.lines if k == 'warn'] == [
        '[planner] LAND stall test: d1 no pose yet: stall count held at 0 while it is stale']
    h.n, h.slot2drone = 1, [0]                             # it leaves the load while stale (a resize)
    LoadPlanner._touchdown_stalled(h)
    assert h.lines[-1] == ('info', '[planner] LAND stall test: d1 left the load after 2 stale tick(s) (0.2 s)')
