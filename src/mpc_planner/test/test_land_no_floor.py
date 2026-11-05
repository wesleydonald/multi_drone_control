"""land_descend_to_stall (card 2026-10-17_land_ref_below_rest): after ring touchdown the straight LAND reference
keeps descending past z 0 until the stall freezes it. A drone 3 % stronger than its thrust map hovered 0.26 m above a
reference stopped at 0 and disarmed in the air (R2088). Off: today's nodes exactly."""
import types

import numpy as np

from mpc_planner.planner_node import LAND_BLEND_S, LAND_FOLLOW_XY_Z, PLANNER_HZ, LoadPlanner, land_arc_ref

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


def _land_hold_refs_reviewed(self):
    """_land_hold_refs as committed on week8 9f20f8b (rev 3's switch, reviewed 17-18 Oct), frozen here: the rig flies
    land_descend_to_stall by default since 5 Nov, so its references are pinned against this copy."""
    if self._land_anchor is None:
        if self._land_rearm:
            self._touchdown.reset()
        self._land_anchor = {}
        for i in range(self.n):
            p = self._drone_at(i)
            self._land_anchor[i] = (float(p[0]), float(p[1]), float(p[2]))
        self._land_arc = self._land_arcs() if self._land_unwind else None
        self._land_drop = 0.0
        self._land_blend_t = 0.0
    if not self._touched_down:
        self._land_drop += self.land_vel / PLANNER_HZ
    self._land_blend_t += 1.0 / PLANNER_HZ
    s = float(np.clip(self._land_blend_t / LAND_BLEND_S, 0.0, 1.0))
    hover = np.array([0.0, 0.0, self.dyn.g])
    for i in range(self.n):
        drone = self.slot2drone[i]
        ax, ay, az = self._land_anchor[i]
        p = self._drone_at(i)
        if p is not None and float(p[2]) < LAND_FOLLOW_XY_Z:
            ax, ay = float(p[0]), float(p[1])
            self._land_anchor[i] = (ax, ay, az)
        a0, c0 = self._last_air_ref.get(drone, (hover, np.zeros(3)))
        a_ff = (1.0 - s) * a0 + s * hover
        c_ff = (1.0 - s) * c0
        nodes = []
        for k in range(self.N + 1):
            if self._land_arc is not None:
                v = 0.0 if self._touched_down else self.land_vel
                pos, vel = land_arc_ref(*self._land_arc[i], self._land_drop + v * self.dt * k, v,
                                        floor=-np.inf if self._land_no_floor else 0.0)
                if p is not None and float(p[2]) < LAND_FOLLOW_XY_Z:
                    pos[0:2], vel[0:2] = (ax, ay), 0.0
                nodes.append((tuple(pos), tuple(vel), tuple(a_ff), tuple(c_ff)))
                continue
            if self._land_no_floor:
                v = 0.0 if self._touched_down else self.land_vel
                z, vz = az - self._land_drop - v * self.dt * k, -v
            else:
                z = max(0.0, az - self._land_drop - self.land_vel * self.dt * k)
                vz = -self.land_vel if (z > 0.0 and not self._touched_down) else 0.0
            nodes.append(((ax, ay, z), (0.0, 0.0, vz), tuple(a_ff), tuple(c_ff)))
        self._publish_ref(drone, nodes)


def test_references_are_the_reviewed_ones():
    """Both LAND paths, with and without the floor, two slots moving down through the xy follow below
    LAND_FOLLOW_XY_Z and the touchdown freeze: node for node the reviewed function's."""
    attach, radial, off = np.array([0.225, 0.0, 0.03]), np.array([1.0, 0.0, 0.0]), np.array([0, 0, 0.04])
    for no_floor in (False, True):
        for arc in (None, {0: (attach, radial, 1.0, np.radians(50.0), off),
                           1: (attach, -radial, 1.0, np.radians(40.0), off)}):
            now, ref = _fake(no_floor), _fake(no_floor)
            for f in (now, ref):
                f.n, f.slot2drone, f._land_unwind, f.zt = 2, [0, 1], arc is not None, [0.80, 0.70]
                f._land_arcs = lambda a=arc: a
                f._drone_at = (lambda g: (lambda i: np.array([1.0 + i + 0.01 * g.zt[i], 0.02 * i, g.zt[i]])))(f)
            for k in range(60):
                if k == 45:
                    now._touched_down = ref._touched_down = True
                for f in (now, ref):
                    f.zt = [max(0.10, z - 0.015) for z in f.zt]
                LoadPlanner._land_hold_refs(now)
                _land_hold_refs_reviewed(ref)
                for d in (0, 1):
                    assert [n[:2] for n in now.sent[d]] == [n[:2] for n in ref.sent[d]]
