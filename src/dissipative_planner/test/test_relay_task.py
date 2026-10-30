"""relay_task (twin only; card 2026-10-30_x4_endurance_relay, critic waived by Wesley 30 Oct): tasks on a lapped path.
A carrier's TASK waits for its leave and starts from its staging point; an announced leave waits until no other carrier
is off the ring (8 -> 7, never 6); the path is held while a task's drone is inside the lap's keep-out (its staging
radius about every point of the lap), so the path runs only while it is clear; the task ends parked at its last
waypoint, and an ATTACH (in a hold) flies it back through the waypoints it only passed to its staging point, where its
rejoin starts. Off: rejoin_task as before (a carrier's TASK refused, a DETACH not queued)."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402
from mpc_planner.planner_node import PLANNER_HZ  # noqa: E402
from test_lap_hold import _dnode, _held_runs, _laps_done, _parent_plan  # noqa: E402
from mpc_planner.planner_node import LoadPlanner  # noqa: E402
from test_rejoin_departed import HZ, _fly, _fly_until, _leave, _see_tip  # noqa: E402
from test_rejoin_task import _said, _task, _tnode  # noqa: E402

RELAY_METHODS = ('_relay_route', '_relay_keep', '_relay_inside', '_relay_route_refused', '_relay_hold', '_relay_park',
                 '_relay_attach_refused', '_relay_attach', '_relay_return', '_relay_return_step', '_relay_away')
PAD1, PAD3 = (0.0, 1.45), (0.0, -1.45)      # clear of the size-0.5 figure-8's keep-out (1.005 m on the 1 m square)


def _rnode(relay=True, laps=9):
    """The 1 m square on a size-0.5 figure-8 of `laps` laps, held at the end of lap 1 (the ring at rest at its start
    point): the path's clock stands, the whole lap still to fly."""
    f = _tnode()
    f._relay_task, f._relay_next, f._relay_leave = relay, {}, None
    for name in RELAY_METHODS:
        setattr(f, name, __import__('types').MethodType(getattr(D, name), f))
    f.traj = LoadTrajectory('fig_8', 0.2, 1.0, 0.5, laps=laps)
    f.traj_t = f.traj.lap_s()
    return f


def _z(f, d=1):
    return float(f.drone_pos[d][2])


def _pad(xy, z, dwell):
    return f'{xy[0]},{xy[1]},{z:.6f},{dwell}'


def _out_and_park(f, d=1, pad=PAD1, dwell=2.0):
    """d's relay task queued and its leave: returns the ticks' (reference, hold after the tick)."""
    _task(f, f'{d} {_pad(pad, _z(f, d), dwell)}')
    f._fleet_detach_cb(SimpleNamespace(data=d))
    seen = []
    for _ in range(800):
        _fly(f, d, 1)
        seen.append((f.published[d][-1][0].copy(), f._reconfig_hold_left))
        if f._rejoin[d]['mode'] == 'park':
            return seen
    raise AssertionError('never parked')


def test_switch_off_a_carriers_task_is_refused_and_a_detach_is_not_queued():
    f = _rnode(relay=False)
    _task(f, f'1 {_pad(PAD1, _z(f), 2)}')
    assert _said(f, 'TASK REFUSED drone 2 (/drone_1)', 'not a staged carrier') and not f._relay_next
    _leave(f, 1)
    f._fleet_detach_cb(SimpleNamespace(data=3))
    assert f.detached[3] and not _said(f, 'RELAY LEAVE QUEUED')            # 4 -> 2: no relay rule


def test_a_carriers_task_waits_for_its_leave_then_flies_out_held_inside_the_keep_out_and_parks():
    f = _rnode()
    _task(f, f'1 {_pad(PAD1, _z(f), 2)}')
    assert _said(f, 'TASK QUEUED drone 2 (/drone_1)') and 1 in f._relay_next and not f.detached[1]
    seen = _out_and_park(f)
    rj = f._rejoin[1]
    assert _said(f, 'TASK START drone 2 (/drone_1)', 'parked at the last') and 1 not in f._relay_next
    keep = f._relay_keep(rj)
    lap = f._task_ring_points()
    inside = [float(np.min(np.linalg.norm(lap - p[:2], axis=1))) < keep for p, _ in seen]
    assert inside[0] and not inside[-1]
    assert all(h > 0.0 for (_, h), i in zip(seen, inside) if i)           # the path held while it is inside
    i_out = inside.index(False)
    assert not any(inside[i_out:])                                         # out once, never back in
    assert all(np.hypot(p[0], p[1]) >= keep - 0.02 for p, _ in seen[i_out:])
    p = np.array([q for q, _ in seen])
    assert (np.linalg.norm(p[:, :2] - PAD1, axis=1) < 0.02).sum() >= 2.0 * HZ   # its dwell held in full
    assert _said(f, 'TASK AT drone 2 (/drone_1) waypoint 1 of 1') and _said(f, 'TASK PARKED drone 2 (/drone_1)')
    _fly(f, 1, 100)
    assert rj['mode'] == 'park' and np.linalg.norm(f.published[1][-1][0][:2] - PAD1) < 0.02   # stays parked


@pytest.mark.parametrize('pads, why', [
    ([((1.2, 0.0), 1)], 'waypoint 1 (1.20, 0.00) comes'),                     # inside the lobe's keep-out
    ([((1.45, 1.45), 0), ((1.45, -1.45), 1)], 'comes back inside'),           # out, then past the lobe tip
    ([((1.45, 1.45), 1), ((1.45, -1.45), 1)], 'from the ring where it is held'),  # back straight across the ring
])
def test_a_route_that_would_meet_the_lap_is_refused_and_the_leaver_stays_staged(pads, why):
    f = _rnode()
    _task(f, '1 ' + '; '.join(_pad(p, _z(f), dw) for p, dw in pads))
    f._fleet_detach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'stage' and _said(f, 'TASK REFUSED drone 2 (/drone_1)', why)


def test_a_relay_task_is_refused_while_the_path_runs():
    f = _rnode()
    _leave(f, 1)
    _fly(f, 1, 60)
    f._traj_running = True
    _task(f, f'1 {_pad(PAD1, _z(f), 1)}')
    assert f._rejoin[1]['mode'] == 'stage' and _said(f, 'TASK REFUSED', 'path is running')


def test_an_attach_while_the_path_runs_is_refused_in_a_hold_it_flies_back_and_rejoins():
    f = _rnode()
    _out_and_park(f, dwell=1.0)
    f._traj_running = True
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'park' and _said(f, 'REJOIN REFUSED drone 2 (/drone_1)', 'path is running')
    f._traj_running = False
    f._fleet_attach_cb(SimpleNamespace(data=1))
    rj = f._rejoin[1]
    assert rj['mode'] == 'return' and _said(f, 'RELAY RETURN drone 2 (/drone_1)')
    stage = f._stage_point(rj)[0]
    back = []
    while f._rejoin.get(1, {}).get('mode') == 'return':
        _fly(f, 1, 1)
        back.append(f.published[1][-1][0].copy())
        assert f._reconfig_hold_left > 0.0                                 # the path held all the way back
        assert len(back) < 600
    assert np.linalg.norm(back[-1] - stage) < 0.03
    assert min(np.hypot(p[0], p[1]) for p in back) >= f._relay_keep(rj) - 0.02
    assert _said(f, 'TASK DONE drone 2 (/drone_1)', 'its rejoin starts') and f._rejoin[1]['mode'] == 'approach'
    f.resizes.clear()
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 600)
    assert f.resizes[-1][0] == 4 and f.resizes[-1][1][-1] == 1            # welded back on its own plate


def test_an_attach_during_its_dwell_waits_for_the_dwell_and_holds_the_path():
    f = _rnode()
    _task(f, f'1 {_pad(PAD1, _z(f), 3.0)}')
    f._fleet_detach_cb(SimpleNamespace(data=1))
    assert _fly_until(f, 1, lambda f: _said(f, 'TASK AT drone 2 (/drone_1)'), 600)
    _fly(f, 1, 5)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    rj = f._rejoin[1]
    assert rj['mode'] == 'task' and rj['attach_queued'] and _said(f, 'RELAY ATTACH QUEUED drone 2 (/drone_1)')
    n0 = len(f.published[1])
    assert _fly_until(f, 1, lambda f: f._rejoin[1]['mode'] == 'return', 100)
    p = np.array([r[0] for r in f.published[1][n0:]])
    assert len(p) >= int(2.4 * HZ) and np.all(np.linalg.norm(p[:-1, :2] - PAD1, axis=1) < 0.02)  # dwell not cut
    assert _said(f, 'TASK PARKED drone 2 (/drone_1)') and f._reconfig_hold_left > 0.0


def test_an_errand_flies_back_through_its_pass_points_only():
    """Out through a pass point (no dwell), a pickup and a delivery (dwells); back from the delivery through the pass
    point to its staging point, not through the pickup."""
    f = _rnode()
    z = _z(f)
    via, pick, drop = (0.0, 1.45), (0.8, 1.5), (-0.8, 1.5)
    _task(f, f'1 {_pad(via, z, 0)}; {_pad(pick, z, 1)}; {_pad(drop, z, 1)}')
    f._fleet_detach_cb(SimpleNamespace(data=1))
    assert _fly_until(f, 1, lambda f: f._rejoin[1]['mode'] == 'park', 1500)
    assert _said(f, 'waypoint 2 of 3') and _said(f, 'waypoint 3 of 3')
    f._fleet_attach_cb(SimpleNamespace(data=1))
    rj = f._rejoin[1]
    assert len(rj['back']) == 1 and np.allclose(rj['back'][0][:2], via)
    n0 = len(f.published[1])
    assert _fly_until(f, 1, lambda f: f._rejoin[1]['mode'] == 'approach', 600)
    p = np.array([r[0] for r in f.published[1][n0:]])
    assert (np.linalg.norm(p[:, :2] - via, axis=1) < 0.03).any()
    assert np.min(np.linalg.norm(p[:, :2] - pick, axis=1)) > 0.3


def test_land_while_parked_lands_it_from_where_it_is():
    f = _rnode()
    _out_and_park(f, dwell=0.5)
    q = f._departed_hold[1].copy()
    f._land_to_ground = True
    f._rejoin_tick()
    assert 1 not in f._rejoin and 1 in f._land_alone
    assert f._departed_step[1][2] == pytest.approx(q[2])                  # no climb: it is not over a plate
    assert _said(f, 'TASK ABORT drone 2 (/drone_1) (LAND)')


def test_a_leave_waits_for_a_full_ring_and_a_failed_rejoin_cancels_it():
    f = _rnode()
    _out_and_park(f, dwell=0.5)
    _see_tip(f, 3, f.now[0])
    _task(f, f'3 {_pad(PAD3, _z(f, 3), 1)}')
    f._fleet_detach_cb(SimpleNamespace(data=3))
    assert not f.detached[3] and f._relay_leave == 3 and _said(f, 'RELAY LEAVE QUEUED drone 4 (/drone_3)')
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert _fly_until(f, 1, lambda f: f._rejoin[1]['mode'] == 'approach', 600)
    f._rejoin_retreat(1, f._rejoin[1], 'test')
    assert f._relay_leave is None and _said(f, 'RELAY LEAVE CANCELLED drone 4 (/drone_3)')
    assert not f.detached[3]


# ── on the real lap clock (test_lap_hold._dnode: _plan_inner with the parent's clock) ─────────────────────────────

@pytest.fixture
def _parent(monkeypatch):
    monkeypatch.setattr(LoadPlanner, '_plan', _parent_plan)


def _relay_dnode(laps=4):
    f = _dnode(laps=laps)
    f._rejoin_task, f._relay_task, f._relay_next, f._relay_leave = True, True, {}, None
    return f


def _tick_all(f):
    """One planner tick; every drone tracks its reference, every rod's end is seen at 60 Hz and the twin's joint
    answers each /drone_d/attach. Returns (clock, the relay drones inside the lap's keep-out while the path ran)."""
    sent = {d: len(f._attach_pubs[d].msgs) for d in range(f._n_carry0)}
    f._plan_inner()
    for d, refs in f.published.items():
        if refs:
            f.drone_pos[d] = refs[-1][0].copy()
    f.now[0] += 1.0 / HZ
    for d in range(f._n_carry0):
        for k in range(6):
            _see_tip(f, d, f.now[0] + k / (6 * HZ), (0.0, 0.0, 1e-7 * (k + 1)))
        if len(f._attach_pubs[d].msgs) > sent[d]:
            f._rejoin_joint_cb(d, SimpleNamespace(data='attached'))
    bad = []
    if f._traj_running:
        for d, rj in f._rejoin.items():
            if rj.get('relay') and f._relay_inside(rj, rj['q']):
                bad.append(d)
    return f.traj_t, bad


def _run(f, done, max_ticks=6000):
    ts, bad = [], []
    while not done(f):
        t, b = _tick_all(f)
        ts.append(t)
        bad += b
        assert len(ts) < max_ticks
    return ts, bad


def test_a_relay_swap_on_the_lap_clock_never_runs_the_path_with_a_relay_drone_inside_and_never_runs_back(_parent):
    """Lap 1 end: drone 1 leaves for its pad (TASK queued, DETACH). It flies out with the path held until clear, the
    lap runs, it parks. Lap 2 end: ATTACH 1, TASK 3, DETACH 3 at once: 3 waits for the full ring, 1 flies back with
    the path held and welds, and at its blend over 3 leaves for its pad. The clock never runs back and never runs
    while a relay drone is inside the lap's keep-out; the ring never has two off."""
    f = _relay_dnode()
    ts, bad = _run(f, lambda f: len(_laps_done(f)) >= 1)
    ts += [_tick_all(f)[0] for _ in range(5)]               # the runner's events come 0.5 s into the hold
    _task(f, f'1 {_pad(PAD1, _z(f, 1), 2)}')
    f._fleet_detach_cb(SimpleNamespace(data=1))
    t1, b1 = _run(f, lambda f: len(_laps_done(f)) >= 2)
    ts, bad = ts + t1, bad + b1
    ts += [_tick_all(f)[0] for _ in range(5)]
    assert _said(f, 'TASK PARKED drone 2 (/drone_1)') and f._rejoin[1]['mode'] == 'park'
    t_lap2 = f.traj_t
    f._fleet_attach_cb(SimpleNamespace(data=1))
    _task(f, f'3 {_pad(PAD3, _z(f, 3), 2)}')
    f._fleet_detach_cb(SimpleNamespace(data=3))
    assert f._relay_leave == 3 and not f.detached[3]
    t2, b2 = _run(f, lambda f: _said(f, 'TASK PARKED drone 4 (/drone_3)'))
    ts, bad = ts + t2, bad + b2
    assert not bad
    assert min(np.diff(ts)) >= -1e-12
    assert f.resizes[-2][0] == 4 and 1 in f.resizes[-2][1]                 # 1 welded back first ...
    assert f.resizes[-1][0] == 3 and 3 not in f.resizes[-1][1]             # ... then 3 left
    assert all(m >= 3 for m, *_ in f.resizes)                              # never two off the ring
    i2 = ts.index(t_lap2)
    assert _held_runs(ts[i2:])[0][0] == 0                                  # held from the lap-2 end ...
    assert ts[-1] > t_lap2                                                 # ... and the path ran again after
