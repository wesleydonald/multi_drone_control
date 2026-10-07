"""Soft detach (7 Oct): with detach_unload_s > 0 the leaver is unloaded in the n-drone OCP and
its magnet opens only after the blend and the gates; a second detach, a failed gate or a LAND
cancels cleanly with the magnet ON. detach_unload_s 0 keeps the one-tick detach."""
import time
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D  # noqa: E402
from mpc_planner.geometry import attach_points, nominal_cable_dirs  # noqa: E402
from mpc_planner.reference_builder import ReferenceBuilder  # noqa: E402


class _Log:
    def __init__(self):
        self.lines = []

    def info(self, m, *a, **k):
        self.lines.append(m)

    warn = error = info


def _node(unload_s=3.0, leaver_t=0.2, tilt_q=(1.0, 0.0, 0.0, 0.0)):
    rho = attach_points(4, 0.225, 0.0, '30,90,150,270')
    s = nominal_cable_dirs(rho, 60.0)
    dyn = SimpleNamespace(rho=rho, m=0.86, g=9.81)
    X = np.zeros((13 + 14 * 4, 21))
    X[13 + 14 * 1 + 12, :] = leaver_t
    log = _Log()
    f = SimpleNamespace(
        slot2drone=[0, 1, 2, 3], detached=[False] * 4, _min_survivors=3, rho=rho, dyn=dyn,
        refs=ReferenceBuilder(dyn, 4, s, 0.1, None), phase='planner', _reconfig_mode='ocp',
        _land_to_ground=False, _unload_s=unload_s, _unload_t=0.2, _unload_tilt=10.0,
        _unload_wait=2.0, _post_blend_s=1.0, _unload=None, _reconfig_hold_left=0.0,
        solver=SimpleNamespace(last_X=X), load_state=np.r_[0.0, 0.0, 1.0, tilt_q, np.zeros(6)])
    f.calls, f.log = [], log
    f.get_logger = lambda: log
    for m in ('_detach_plan', '_start_unload', '_cancel_unload', '_unload_tick', '_unload_cancel_on'):
        setattr(f, m, (lambda name: lambda *a, **k: getattr(D, name)(f, *a, **k))(m))
    f._detach_ocp = lambda d: f.calls.append(('plain', d))
    f._resize_and_release = lambda d, *plan, post_blend=None: f.calls.append(('release', d, post_blend)) or True
    return f


def _detach(f, d):
    D._fleet_detach_cb(f, SimpleNamespace(data=d))


def _run_blend(f, ticks=30):
    for _ in range(ticks):
        f._unload_tick()
        f.refs.update((0, 0), 0.0, 0.0, 1.0, 0.0, 0.0)


def test_unload_zero_keeps_the_one_tick_detach():
    f = _node(unload_s=0.0)
    _detach(f, 1)
    assert f.calls == [('plain', 1)] and f._unload is None


def test_no_release_until_the_blend_is_done_then_one_release():
    f = _node()
    _detach(f, 1)
    assert f._unload is not None and f.calls == [] and f.refs.blend_active()
    assert f._reconfig_hold_left > 3.0
    _run_blend(f, 29)
    assert f.calls == []
    _run_blend(f, 5)
    assert [c[0] for c in f.calls] == ['release'] and f._unload is None
    t_from, T = f.calls[0][2]
    assert len(t_from) == 3 and T == 1.0


def test_second_detach_during_an_unload_is_refused():
    f = _node()
    _detach(f, 1)
    _detach(f, 2)
    _detach(f, 1)
    assert f._unload['d'] == 1 and f.calls == []


def test_gates_not_met_cancel_with_the_magnet_on():
    f = _node(leaver_t=1.5)                       # the planner never unloads the leaver
    _detach(f, 1)
    t_pre = list(f._unload['t_pre'])
    _run_blend(f, 60)
    assert f.calls == [] and f._unload is None
    assert any('CANCELLED' in m for m in f.log.lines)
    _run_blend(f, 35)                                  # back over the unload's 3 s
    assert np.allclose(f.refs._refs_at(0)[1], t_pre)   # blended back to the full split


def test_land_during_an_unload_cancels():
    f = _node()
    _detach(f, 1)
    f._unload_cancel_on('land')
    assert f._unload is None and f.calls == []


def _detect_node(dists, detect_m=0.06):
    log = _Log()
    f = SimpleNamespace(_detect_m=detect_m, phase='planner', _reconfig_mode='ocp', _land_to_ground=False,
                        _unload=None, takeoff_seen=True, slot2drone=[0, 1, 2, 3], detached=[False] * 4,
                        cable_len=0.55, cable_len_i=[0.55] * 4, _detect_count={}, _detect_ok={},
                        _detect_refused=set(), calls=[], _load_t=None, _drone_t={})
    f.fresh = lambda: (setattr(f, '_load_t', time.monotonic()),
                       f._drone_t.update({k: time.monotonic() for k in range(4)}))
    f.get_logger = lambda: log
    f._rim_dist = lambda i: dists[i]
    def _det(d):
        f.calls.append(d)
        f.detached[d] = True
    f._detach_ocp = _det
    return f


def test_unannounced_detach_needs_two_ticks_over_the_length():
    dists = [0.55, 0.55, 0.55, 0.55]
    f = _detect_node(dists)
    f.fresh(); D._detect_tick(f)
    dists[1] = 0.65                      # drone 1's cable 10 cm over: its magnet let go
    f.fresh(); D._detect_tick(f)
    assert f.calls == []                 # one tick is not enough (a mocap glitch)
    f.fresh(); D._detect_tick(f)
    assert f.calls == [1]


def test_detection_off_or_one_glitch_does_nothing():
    dists = [0.55, 0.70, 0.55, 0.55]
    f = _detect_node(dists, detect_m=0.0)
    for _ in range(3):
        f.fresh(); D._detect_tick(f)
    assert f.calls == []
    f = _detect_node(dists)
    f.fresh(); D._detect_tick(f)
    dists[1] = 0.55                      # back within its length: the count restarts
    f.fresh(); D._detect_tick(f)
    dists[1] = 0.70
    f.fresh(); D._detect_tick(f)
    assert f.calls == []


def test_a_rod_measured_short_or_a_steady_bias_is_not_a_release():
    dists = [0.55, 0.55, 0.55, 0.55]
    f = _detect_node(dists)
    f.cable_len_i = [0.48, 0.48, 0.48, 0.48]     # r001: rods measured 6-7 cm short at the handover
    for _ in range(5):
        f.fresh(); D._detect_tick(f)
    assert f.calls == []
    dists[2] = 0.63                              # a steady 8 cm bias from the first tick
    f = _detect_node(dists)
    for _ in range(5):
        f.fresh(); D._detect_tick(f)
    assert f.calls == []


def test_r200006_slip_fires_on_two_equal_stale_ticks():
    dists = [0.55, 0.55, 0.55, 0.55]
    f = _detect_node(dists)
    f.fresh(); D._detect_tick(f)
    dists[3] = 0.725                             # +0.175 on two ticks with the same mocap sample
    f.fresh(); D._detect_tick(f)
    f.fresh(); D._detect_tick(f)
    assert f.calls == [3]


def test_departed_land_steps_out_then_descends_while_the_fleet_flies():
    published = []
    f = SimpleNamespace(dyn=SimpleNamespace(g=9.81), _land_to_ground=False, _departed_land=True,
                        _departed_landing=set(), _departed_td={}, _departed_clear={},
                        _departed_land_vel=0.0, land_vel=0.15, N=20, load_state=np.zeros(13),
                        _departed_hold={1: np.array([0.5, 0.0, 1.5])},
                        _departed_step={1: np.array([1.0, 0.0, 1.5])}, _departed_next={},
                        drone_pos=[None, None])
    f.get_logger = lambda: _Log()
    f._publish_ref = lambda d, nodes: published.append(nodes[0][0].copy())
    for _ in range(40):                          # 0.5 m at 0.15 m/s: done in ~3.3 s
        D._publish_departed_refs(f)
    assert np.allclose(published[-1][:2], [1.0, 0.0], atol=0.02)
    z0 = published[-1][2]
    for _ in range(10):
        D._publish_departed_refs(f)
    assert published[-1][2] < z0 - 0.2 and np.allclose(published[-1][:2], [1.0, 0.0], atol=0.02)
    assert 1 in f._departed_landing


def test_path_clear_point_during_a_circle():
    # orbit r 0.5 from the hover point (0, 0): the path centre is (0, 0.5), the clear radius 1.5
    f = SimpleNamespace(traj=SimpleNamespace(kind='orbit', radius=0.5), hover_xy=(0.0, 0.0))
    up, east = np.array([0.0, 1.0]), np.array([1.0, 0.0])
    assert np.allclose(D._path_clear_point(f, np.array([0.0, 1.0, 1.5]), up), [0.0, 2.0, 1.5])
    assert np.allclose(D._path_clear_point(f, np.array([0.3, 0.1, 1.4]), up),
                       [0.9, -0.7, 1.4])                       # radial from the centre, not the ring step
    assert np.allclose(D._path_clear_point(f, np.array([0.0, 0.52, 1.5]), east), [1.5, 0.5, 1.5])
    assert D._path_clear_point(f, np.array([0.0, 2.1, 1.5]), up) is None         # already clear
    for kind in ('hover', 'fig_8', 'line_x'):
        f.traj.kind = kind
        assert D._path_clear_point(f, np.array([0.0, 1.0, 1.5]), up) is None
    f.traj.kind, f.hover_xy = 'circle', None
    assert D._path_clear_point(f, np.array([0.0, 1.0, 1.5]), up) is None


def test_freed_drone_clears_the_circle_before_it_lands():
    published = []
    f = SimpleNamespace(dyn=SimpleNamespace(g=9.81), _land_to_ground=False, _departed_land=True,
                        _departed_landing=set(), _departed_td={}, _departed_clear={},
                        _departed_land_vel=0.0, land_vel=0.15, N=20, load_state=np.zeros(13),
                        _departed_hold={3: np.array([0.0, 0.5, 1.5])},
                        _departed_step={3: np.array([0.0, 1.0, 1.5])},
                        _departed_next={3: np.array([0.0, 2.0, 1.5])}, drone_pos=[None] * 4)
    f.get_logger = lambda: _Log()
    f._publish_ref = lambda d, nodes: published.append(nodes[0][0].copy())
    for _ in range(40):                          # the 0.5 m ring step (3.3 s): no landing yet
        D._publish_departed_refs(f)
    assert 3 not in f._departed_landing and published[-1][1] > 1.0 and published[-1][2] == 1.5
    for _ in range(75):                          # then 1.0 m more to the path-clear point
        D._publish_departed_refs(f)
    assert np.allclose(published[-1][:2], [0.0, 2.0], atol=0.02) and 3 in f._departed_landing
    assert all(q[2] == 1.5 for q in published if q[1] < 1.98)    # level until clear
    assert published[-1][2] < 1.5 and not f._departed_next


def test_stale_ring_or_drone_pose_or_all_cables_over_is_not_a_release():
    dists = [0.55, 0.55, 0.55, 0.55]
    f = _detect_node(dists)
    f.fresh(); D._detect_tick(f)
    dists[1] = 0.70
    f._load_t = time.monotonic() - 0.5           # ring pose 0.5 s old
    for _ in range(3):
        D._detect_tick(f)
    assert f.calls == []
    f = _detect_node([0.70, 0.70, 0.70, 0.70])   # every cable over at once: a ring pose jump
    for _ in range(4):
        f.fresh(); D._detect_tick(f)
    assert f.calls == []
    dists = [0.55, 0.55, 0.55, 0.55]
    f = _detect_node(dists)
    f.fresh(); D._detect_tick(f)
    dists[2] = 0.70
    for _ in range(3):                           # drone 2's own pose frozen
        f.fresh(); f._drone_t[2] = time.monotonic() - 0.5; D._detect_tick(f)
    assert f.calls == []
