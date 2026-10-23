"""traj_lap_hold_s (twin only; card 2026-10-23_scalability case 7, critic waived by Wesley 23 Oct): on a lapped path the
trajectory clock is held at each lap end but the last, through the reconfiguration hold, and 'lap k done' is logged.
Two clock holds overlapping ran the clock backwards (the hold's cancel takes a tick off whether or not the parent
advanced it, R0518): the lap-end hold is the same one counter as a resize's and a rejoin's, so a rejoin started in it
holds the path on and the clock never runs back. These ticks are the real _plan_inner with the parent's clock (its
pre-path hold, advance and completion) standing in for LoadPlanner._plan. Off: the clock runs through every lap end as
before."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402
from mpc_planner.planner_node import LoadPlanner, PLANNER_HZ  # noqa: E402
from test_rejoin_departed import HZ, METHODS, _leave, _node, _Refs, _see_tip  # noqa: E402

QUIET = ('_contact_release_tick', '_release_seen_tick', '_land_take_back_tick', '_newcomer_off_tick', '_announce_on_load',
         '_partner_release_tick', '_publish_pending_attach_refs', '_partner_magnet_keepalive', '_contact_span_tick',
         '_unload_tick', '_detect_tick')


def _parent_plan(self):
    """LoadPlanner._plan's trajectory clock: its pre-path hold, then one tick a tick until the path completes."""
    if self.descending:
        return
    if self._traj_hold_left > 0.0:
        self._traj_hold_left -= 1.0 / PLANNER_HZ
    else:
        self.traj_t += 1.0 / PLANNER_HZ
        if self.traj.complete(self.traj_t):
            self.descending = True


@pytest.fixture(autouse=True)
def _parent(monkeypatch):
    monkeypatch.setattr(LoadPlanner, '_plan', _parent_plan)


def _dnode(hold_s=3.0, laps=3):
    """The rejoin tests' 4-quad square (test_rejoin_departed._node) on a real planner instance, flying a size-0.5
    figure-8 of `laps` laps from the end of the lift."""
    ns = _node()
    f = D.__new__(D)
    for k, v in ns.__dict__.items():
        if k not in METHODS:
            setattr(f, k, v)

    def resize(m, ids, rho=None, cable_lengths=None):
        f.resizes.append((m, list(ids), [np.array(r, float) for r in rho], list(cable_lengths or [])))
        f.n, f.slot2drone, f.rho = m, list(ids), [np.array(r, float) for r in rho]
        if cable_lengths is not None:
            f.cable_len_i = list(cable_lengths)
        f.dyn = SimpleNamespace(g=9.81, rho=list(f.rho), m=0.86)
        f._s_nom = [np.array([-r[0], -r[1], -0.39]) / np.linalg.norm([r[0], r[1], 0.39]) for r in f.rho]
        f.refs = _Refs(m)
        return True
    f.resize_fleet = resize
    for name in QUIET:
        setattr(f, name, lambda: None)
    f._auto_handover_due = lambda: False
    f.load_traj, f.traj = 'fig_8', LoadTrajectory('fig_8', 0.2, 1.0, 0.5, laps=laps)
    f._traj_hold_left, f._traj_t_tick, f._settle_hold, f._resize_log_ticks = 0.0, 0.0, False, 0
    f._lap_hold_s, f._lap_held = hold_s, 0
    return f


def _tick(f, d=None):
    """One planner tick; d, if given, tracks its reference and its rod's end is seen at 60 Hz (its joint answers each
    /drone_d/attach). Returns the clock after the tick."""
    sent = len(f._attach_pubs[d].msgs) if d is not None else 0
    f._plan_inner()
    if d is not None and f.published.get(d):
        f.drone_pos[d] = f.published[d][-1][0].copy()
    f.now[0] += 1.0 / HZ
    if d is not None:
        for k in range(6):
            _see_tip(f, d, f.now[0] + k / (6 * HZ), (0.0, 0.0, 1e-7 * (k + 1)))
        if len(f._attach_pubs[d].msgs) > sent:
            f._rejoin_joint_cb(d, SimpleNamespace(data='attached'))
    return f.traj_t


def _until(f, done, d=None, max_ticks=4000):
    ts = []
    while not done(f):
        ts.append(_tick(f, d))
        assert len(ts) < max_ticks
    return ts


def _laps_done(f):
    return [m for m in f.log.lines if ' done at t=' in m]


def _held_runs(ts):
    """(first index, ticks) of each run of ticks the clock stood still."""
    still = np.isclose(np.diff(ts), 0.0, atol=1e-12)
    runs, i = [], 0
    while i < len(still):
        if still[i]:
            j = i
            while j < len(still) and still[j]:
                j += 1
            runs.append((i, j - i))
            i = j
        else:
            i += 1
    return runs


def test_off_the_clock_runs_through_every_lap_end_as_before():
    f = _dnode(hold_s=0.0)
    T = f.traj.lap_s()
    ts = _until(f, lambda f: f.descending)
    assert np.allclose(np.diff(ts), 1.0 / PLANNER_HZ) and ts[-1] == pytest.approx(3 * T, abs=0.1)
    assert not _laps_done(f) and f._reconfig_hold_left == 0.0
    t0 = f.traj_t                       # as before: a hold in the descent takes a tick off the clock each tick
    f._reconfig_hold_left = 0.5
    for _ in range(5):
        _tick(f)
    assert f.traj_t == pytest.approx(t0 - 0.5)


def test_each_lap_end_but_the_last_holds_the_path_once_and_says_so():
    f = _dnode()
    T = f.traj.lap_s()
    ts = _until(f, lambda f: f.descending)
    assert min(np.diff(ts)) >= -1e-12                                       # never backwards
    runs = _held_runs(ts)
    assert len(runs) == 2 and all(n == 30 for _, n in runs)                 # 3 s at each of the two lap ends
    for (i, _), k in zip(runs, (1, 2)):
        assert k * T <= ts[i] < k * T + 1.0 / PLANNER_HZ                   # just past the lap end: the ring at rest
        assert np.allclose(f.traj.offset_at(ts[i]), 0.0, atol=1e-4)
    lines = _laps_done(f)
    assert len(lines) == 2 and 'lap 1 done' in lines[0] and 'lap 2 done' in lines[1] and 'held 3.0 s' in lines[0]
    assert ts[-1] == pytest.approx(3 * T, abs=0.1) and f.traj.complete(ts[-1])


def test_a_rejoin_started_in_the_lap_end_hold_holds_the_one_clock_and_it_never_runs_back():
    """The overlap the design warns of: the lap-end hold (3 s), a rejoin started 0.5 s into it (its approach renews
    the hold every tick), its weld (the fold-in's 10 s), its blend. One counter, so the clock stands at the lap end
    until the last of them, then runs on from there."""
    f = _dnode()
    _leave(f, 1)                                            # released before the path; staged on the moving ring
    ts = _until(f, lambda f: _laps_done(f), d=1)
    t_lap = f.traj_t
    ts += [_tick(f, 1) for _ in range(5)]
    assert f.traj_t == t_lap and not f._traj_running
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'approach'               # the path stands: the rejoin may start
    ts += _until(f, lambda f: 1 not in f._rejoin, d=1, max_ticks=600)
    assert f.resizes[-1][0] == 4 and f.traj_t == t_lap      # welded and folded in 3 s past the lap-end hold's end
    assert len(ts) - ts.index(t_lap) > 3.0 * HZ + 10
    f.refs.active = True                                    # its fold-in blend
    ts += [_tick(f, 1) for _ in range(int(10 * HZ))]
    f.refs.active = False
    ts += _until(f, lambda f: f.traj_t > t_lap, d=1)
    assert f.traj_t == pytest.approx(t_lap + 1.0 / PLANNER_HZ)
    ts += _until(f, lambda f: f.descending, d=1)
    assert min(np.diff(ts)) >= -1e-12                       # through every hold, never backwards
    assert len(_laps_done(f)) == 2 and ts[-1] == pytest.approx(3 * f.traj.lap_s(), abs=0.1)


def test_land_in_a_lap_end_hold_stops_the_clock_where_it_is():
    f = _dnode()
    _until(f, lambda f: _laps_done(f))
    t_lap = f.traj_t
    f.descending = f._land_to_ground = True                 # LAND: the parent's clock stops
    for _ in range(40):
        _tick(f)
    assert f.traj_t == t_lap and len(_laps_done(f)) == 1


def test_the_hazard_the_one_counter_avoids():
    """Held instead on the parent's pre-path hold, the same lap end with a rejoin started in it runs the clock back a
    tick each tick of the rejoin's hold (R0518's mechanism), which is why the lap-end hold is the resize hold."""
    f = _dnode(hold_s=0.0)
    _leave(f, 1)
    T = f.traj.lap_s()
    _until(f, lambda f: f.traj_t >= T, d=1)
    t_lap = f.traj_t
    f._traj_hold_left = 3.0                                 # the lap end on the parent's hold
    _tick(f, 1)
    _tick(f, 1)
    assert f.traj_t == t_lap and not f._traj_running
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'approach'
    for _ in range(10):
        _tick(f, 1)
    assert f.traj_t == pytest.approx(t_lap - 1.0)
