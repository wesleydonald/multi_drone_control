"""rejoin_task (twin only; card 2026-10-23_scalability case 6, critic waived by Wesley 23 Oct): /fleet/task
'd x,y,z,dwell; ...' flies staged carrier d to each waypoint at the approach's 0.2 m/s, holds it there its dwell, and
back to its staging point, where an ATTACH may fly it back to its plate. Refused as a rejoin is, outside the staging
bound, with the rod's end near the floor, or on a route that comes inside its staging radius of a planned ring point.
LAND or a drop lands it from where it is. Off: /fleet/task is refused and the staged carrier holds."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from dissipative_planner.dissipative_node import DissipativeController as D, parse_task  # noqa: E402
from mpc_planner.load_trajectory import LoadTrajectory  # noqa: E402
from test_rejoin_departed import HZ, OUT_R, _fly, _fly_until, _leave, _node, _see_tip  # noqa: E402

TASK_METHODS = ('_fleet_task_cb', '_task_ring_points', '_task_refused', '_start_task', '_task_goal', '_task_step')


def _tnode(task=True, **kw):
    f = _node(**kw)
    f._rejoin_task = task
    for name in TASK_METHODS:
        setattr(f, name, MethodType(getattr(D, name), f))
    return f


def _staged(d=1, **kw):
    f = _tnode(**kw)
    _leave(f, d)
    _fly(f, d, 60)                      # on its staging point (plate 1: +y, 1.025 m out on the 1 m square)
    return f


def _task(f, text):
    f._fleet_task_cb(SimpleNamespace(data=text))


def _said(f, *parts):
    return any(all(p in m for p in parts) for m in f.log.lines)


def test_the_task_text():
    assert parse_task('7 1.45,-1.45,1.93,15') == (7, [(1.45, -1.45, 1.93, 15.0)])
    assert parse_task(' 3 1,2,3,0; -1,-2,3,4.5 ;') == (3, [(1.0, 2.0, 3.0, 0.0), (-1.0, -2.0, 3.0, 4.5)])
    for bad in ('', '7', 'x 1,2,3,4', '7 1,2,3', '7 1,2,3,-1', '7 1,2,nan,1', '7.0 1,2,3,4'):
        with pytest.raises(ValueError):
            parse_task(bad)


def test_switch_off_a_task_is_refused_and_the_carrier_holds_its_staging_point():
    f = _staged(task=False)
    q = f._rejoin[1]['q'].copy()
    _task(f, f'1 0.6,1.45,{q[2]:.4f},2')
    assert f._rejoin[1]['mode'] == 'stage' and _said(f, 'TASK REFUSED drone 2 (/drone_1): rejoin_task is off')
    _fly(f, 1, 20)
    assert f._rejoin[1]['mode'] == 'stage'
    assert np.linalg.norm(f._rejoin[1]['q'] - f._stage_point(f._rejoin[1])[0]) < 1e-3


def test_a_task_visits_each_waypoint_holds_its_dwell_and_comes_back_to_stage():
    f = _staged()
    rj = f._rejoin[1]
    z, stage = rj['z'], f._stage_point(rj)[0]
    w1, w2 = np.array([0.6, 1.45, z]), np.array([-0.6, 1.45, z])
    _task(f, f'1 0.6,1.45,{z:.6f},2; -0.6,1.45,{z:.6f},0')
    assert rj['mode'] == 'task' and _said(f, 'TASK START drone 2 (/drone_1)')
    n0 = len(f.published[1])
    assert _fly_until(f, 1, lambda f: f._rejoin[1]['mode'] == 'stage', 600)
    p = np.array([r[0] for r in f.published[1][n0:]])
    v = np.array([r[1] for r in f.published[1][n0:]])
    assert (np.linalg.norm(p - w1, axis=1) < 0.02).sum() >= 2.0 * HZ              # held there its dwell
    assert (np.linalg.norm(p - w2, axis=1) < 0.02).any()
    assert np.linalg.norm(p[-1] - stage) < 0.02 and 'task' not in rj               # back on its staging point
    assert np.max(np.linalg.norm(v, axis=1)) <= 0.2 + 1e-9                         # the approach's 0.2 m/s
    assert np.max(np.linalg.norm(np.diff(v, axis=0), axis=1)) <= 0.25 / HZ + 1e-9  # TAKE_BACK_ACCEL
    assert np.hypot(p[:, 0], p[:, 1]).min() >= rj['r_stage'] - 0.02                # never inside its staging radius
    steps = [m.split('] ')[1].split(' drone')[0] for m in f.log.lines if '] TASK ' in m]
    assert steps == ['TASK START', 'TASK AT', 'TASK AT', 'TASK DONE']


@pytest.mark.parametrize('why, text, setup', [
    ('outside the staging bound', '1 1.6,1.45,{z},1', None),       # the bound: 1.57 m
    ("rod's end under", '1 0.6,1.45,1.2,1', None),
    ('route comes', '1 0.0,-1.45,{z},1', None),                    # across the ring
    ('LAND', '1 0.6,1.45,{z},1', lambda f: setattr(f, '_land_to_ground', True)),
    ('descent', '1 0.6,1.45,{z},1', lambda f: setattr(f, 'descending', True)),
    ('dropped', '1 0.6,1.45,{z},1', lambda f: setattr(f, '_dropped', 'test')),
    ('not up yet', '1 0.6,1.45,{z},1', lambda f: setattr(f, 'lift_progress', 0.5)),
    ('not a staged carrier', '0 0.6,1.45,{z},1', None),            # drone 0 is on the ring
    ('in its approach', '1 0.6,1.45,{z},1', lambda f: f._fleet_attach_cb(SimpleNamespace(data=1))),
    ('want x,y,z,dwell', '1 0.6,1.45,{z}', None),
])
def test_a_task_is_refused_when_it_cannot_fly(why, text, setup):
    f = _staged()
    mode = f._rejoin[1]['mode'] if setup is None else None
    if setup is not None:
        setup(f)
        mode = f._rejoin[1]['mode']
    _task(f, text.format(z=f'{f._rejoin[1]["z"]:.4f}'))
    assert f._rejoin[1]['mode'] == mode and _said(f, 'TASK REFUSED', why)


def test_an_attach_is_refused_during_the_task_and_flies_it_home_after_task_done():
    f = _staged()
    z = f._rejoin[1]['z']
    _task(f, f'1 0.6,1.45,{z:.6f},1')
    _fly(f, 1, 5)
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'task' and _said(f, 'REJOIN REFUSED drone 2 (/drone_1)', 'already in its task')
    assert _fly_until(f, 1, lambda f: f._rejoin[1]['mode'] == 'stage', 600)
    assert _said(f, 'TASK DONE drone 2 (/drone_1)')
    f.resizes.clear()
    f._fleet_attach_cb(SimpleNamespace(data=1))
    assert f._rejoin[1]['mode'] == 'approach'
    assert _fly_until(f, 1, lambda f: 1 not in f._rejoin, 400)
    assert f.resizes[-1][0] == 4 and f.resizes[-1][1][-1] == 1             # welded back on its own plate


@pytest.mark.parametrize('ticks', [2, 400])
def test_land_mid_task_lands_it_from_where_it_is_with_no_climb(ticks):
    """Near its staging point it first goes out to the clear radius along its own bearing; at its waypoint, already
    past it, it comes down where it is."""
    f = _staged()
    z = f._rejoin[1]['z']
    _task(f, f'1 0.6,1.45,{z:.6f},60')
    _fly(f, 1, ticks)
    assert f._rejoin[1]['mode'] == 'task'
    q = f._departed_hold[1].copy()
    f._land_to_ground = True
    f._rejoin_tick()
    assert 1 not in f._rejoin and 1 in f._land_alone and 1 not in f._departed_next
    step = f._departed_step[1]
    r = np.hypot(*q[:2])
    assert step[2] == pytest.approx(q[2])                                  # no climb over a plate
    assert np.allclose(step[:2], max(r, OUT_R) * q[:2] / r)                # out along its own bearing, or where it is
    assert _said(f, 'TASK ABORT drone 2 (/drone_1) (LAND)')
    for _ in range(300):
        f._publish_departed_refs()
    assert 1 in f._departed_landing and f._departed_hold[1][2] < q[2]


def test_a_drop_mid_task_lands_it_clear_of_the_ring():
    f = _staged()
    _task(f, f'1 0.6,1.45,{f._rejoin[1]["z"]:.6f},5')
    _fly(f, 1, 3)
    f._drop_and_land('test')
    assert 1 not in f._rejoin and 1 in f._land_alone
    assert np.hypot(*f._departed_step[1][:2]) >= OUT_R - 1e-9
    assert _said(f, 'TASK ABORT drone 2 (/drone_1) (drop)')


def test_a_carrier_on_its_task_lets_a_detach_and_another_rejoin_through():
    """The quad on its task is away from the ring, outside its staging radius: one rejoin at a time still holds for
    the others, and a DETACH need not wait for it."""
    f = _tnode()
    _leave(f, 1)
    _leave(f, 3)
    _fly(f, 1, 60)
    _see_tip(f, 3, f.now[0])
    _task(f, f'1 0.6,1.45,{f._rejoin[1]["z"]:.6f},5')
    assert f._rejoin[1]['mode'] == 'task' and f._rejoin_busy() is None
    f._fleet_attach_cb(SimpleNamespace(data=3))
    assert f._rejoin[3]['mode'] == 'approach'
    assert f._rejoin_busy() is not None                                    # the rejoin does hold a DETACH


def test_on_a_path_the_keep_out_takes_every_ring_point_still_to_fly():
    """The figure-8's right lobe (x 0.5, y +-0.25) passes 0.84 m from this route's start: refused there, flown in
    hover."""
    f = _tnode()
    f.traj = LoadTrajectory('fig_8', 0.2, 1.0, 0.5)
    _leave(f, 1)
    _fly(f, 1, 60)
    text = f'1 0.6,1.45,{f._rejoin[1]["z"]:.6f},1'
    _task(f, text)
    assert f._rejoin[1]['mode'] == 'stage' and _said(f, 'TASK REFUSED drone 2 (/drone_1)', 'route comes 0.84 m')
    f.traj = LoadTrajectory('hover', 0.2, 1.0, 0.5)
    _task(f, text)
    assert f._rejoin[1]['mode'] == 'task'
