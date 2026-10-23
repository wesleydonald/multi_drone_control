"""land_descend_to_stall (card 2026-10-17_land_ref_below_rest): after ring touchdown the straight LAND reference
keeps descending past z 0 until the stall freezes it. A drone 3 % stronger than its thrust map hovered 0.26 m above a
reference stopped at 0 and disarmed in the air (R2088). Off: today's nodes exactly."""
import types

import numpy as np

from mpc_planner.planner_node import LoadPlanner

N, DT, LAND_VEL = 10, 0.1, 0.2


def _fake(no_floor, z0=0.30, touched=False):
    sent = {}
    f = types.SimpleNamespace(
        n=1, N=N, dt=DT, land_vel=LAND_VEL, slot2drone=[0], _land_anchor=None, _land_rearm=True,
        _land_no_floor=no_floor, _land_unwind=False, _touched_down=touched, _last_air_ref={},
        dyn=types.SimpleNamespace(g=9.81), _touchdown=types.SimpleNamespace(reset=lambda: None),
        get_logger=lambda: types.SimpleNamespace(info=lambda *a: None))
    f._drone_at = lambda i: np.array([1.0, 0.0, z0])
    f._land_arcs = lambda: None
    f._publish_ref = lambda d, nodes: sent.__setitem__(d, nodes)
    f.sent = sent
    return f


def _ticks(f, k):
    for _ in range(k):
        LoadPlanner._land_hold_refs(f)
    return f.sent[0]


def test_off_stops_at_zero():
    nodes = _ticks(_fake(False), 30)               # 30 ticks: 0.6 m of drop from 0.30
    assert nodes[0][0][2] == 0.0 and nodes[0][1][2] == 0.0
    assert min(n[0][2] for n in nodes) == 0.0


def test_on_keeps_descending_below_zero():
    nodes = _ticks(_fake(True), 30)
    assert np.isclose(nodes[0][0][2], 0.30 - 30 * LAND_VEL * 0.1)
    assert nodes[0][1][2] == -LAND_VEL
    assert np.isclose(nodes[-1][0][2], nodes[0][0][2] - LAND_VEL * DT * N)


def test_on_before_zero_matches_off():
    a, b = _ticks(_fake(False, z0=1.0), 3), _ticks(_fake(True, z0=1.0), 3)
    assert [n[0] for n in a] == [n[0] for n in b] and [n[1] for n in a] == [n[1] for n in b]


def test_on_freezes_at_touchdown():
    f = _fake(True)
    _ticks(f, 5)
    f._touched_down = True
    nodes = _ticks(f, 5)
    assert all(np.isclose(n[0][2], 0.30 - 5 * LAND_VEL * 0.1) and n[1][2] == 0.0 for n in nodes)
