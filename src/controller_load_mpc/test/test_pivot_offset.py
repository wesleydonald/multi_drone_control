"""The rod pivot model (W9, plan 2026-10). On the rig the rod hangs from a joint 4 cm
below the drone centre; the OCP's drone is the rod end. With pivot_offset set, every rod
the planner measures is measured from the pivot, and every position it publishes is the
drone centre the tracker flies. Synthetic geometry: rods of length L from pivots 4 cm
below the centres. With a zero offset nothing changes."""
import types

import numpy as np
import pytest

from controller_load_mpc.planner_node import LoadPlanner, measured_rod_lengths
from controller_load_mpc.planner_solver import PlannerSolver
from controller_load_mpc.creep_controller import CreepController, CREEP_LEAD
from controller_load_mpc.geometry import quat_to_rot_np, centre_from_pivot, thrust_attitude

L = 0.55
B = np.array([0.0, 0.0, -0.04])
G = 9.81


def _roll_quat(deg):
    a = np.deg2rad(deg) / 2
    return np.array([np.cos(a), np.sin(a), 0.0, 0.0])


def _ring(n=4, r=0.225, elev_deg=45.0, load_z=0.02, quats=None):
    load = np.zeros(13)
    load[2] = load_z
    load[3] = 1.0
    rho, pivots, centres = [], [], []
    for k, az in enumerate(np.linspace(0, 2 * np.pi, n, endpoint=False)):
        radial = np.array([np.cos(az), np.sin(az), 0.0])
        rho.append(r * radial)
        e = np.deg2rad(elev_deg)
        piv = load[0:3] + r * radial + L * (np.cos(e) * radial + np.array([0, 0, np.sin(e)]))
        q = quats[k] if quats is not None else np.array([1.0, 0, 0, 0])
        pivots.append(piv)
        centres.append(piv - quat_to_rot_np(q) @ B)
    return load, rho, pivots, centres


def _node(load, rho, centres, pivot_offset, quats=None, slot2drone=None):
    p = LoadPlanner.__new__(LoadPlanner)
    n = len(rho)
    p.n, p.rho, p.load_state = n, rho, load
    p.slot2drone = list(range(n)) if slot2drone is None else slot2drone
    # physical drone slot2drone[i] sits at slot i's centre
    p.drone_pos = [None] * n
    p.drone_quat = {}
    for i, d in enumerate(p.slot2drone):
        p.drone_pos[d] = centres[i]
        p.drone_quat[d] = quats[i] if quats is not None else np.array([1.0, 0, 0, 0])
    p.pivot_offset = np.asarray(pivot_offset, float)
    p.cable_len_i = [L] * n
    return p


@pytest.mark.parametrize('quats', [None, [_roll_quat(15), _roll_quat(-10),
                                          _roll_quat(5), _roll_quat(0)]])
def test_measured_rod_is_L_and_the_taut_gate_is_1(quats):
    load, rho, pivots, centres = _ring(quats=quats)
    p = _node(load, rho, centres, B, quats=quats, slot2drone=[2, 0, 3, 1])
    for i in range(4):
        assert np.allclose(p._pivot_at(i), pivots[i])
        assert p._rim_dist(i) == pytest.approx(L, abs=1e-12)
        assert p._cable_taut_gate(i)[0] == pytest.approx(1.0)
    lens, why = measured_rod_lengths(0.55, [p._rim_dist(i) for i in range(4)], 0.25, 0.08)
    assert why is None and lens == pytest.approx([L] * 4)


def test_without_the_pivot_model_the_rods_read_long():
    load, rho, pivots, centres = _ring()
    p = _node(load, rho, centres, np.zeros(3))
    assert all(p._rim_dist(i) > L + 0.02 for i in range(4))      # the rig's 3-4 cm


def _kin_solver(pivot, acc, offset):
    s = PlannerSolver.__new__(PlannerSolver)
    s._geom = np.zeros(3)
    s.pivot_offset = np.asarray(offset, float)
    s.pos_fun = [lambda x, g: pivot]
    s.vel_fun = [lambda x, g: np.zeros(3)]
    s.acc_fun = [lambda x, g: acc]
    s.cable_fun = [lambda x, g: np.zeros(3)]
    return s


@pytest.mark.parametrize('tilt_deg,yaw', [(0.0, 0.0), (20.0, 0.7)])
def test_published_reference_is_the_centre(tilt_deg, yaw):
    pivot = np.array([0.5, 0.1, 0.6])
    t = np.deg2rad(tilt_deg)
    acc = G * np.array([np.sin(t), 0.0, np.cos(t)]) / np.cos(t)
    pos, _v, a, _c = _kin_solver(pivot, acc, B).drone_kinematics(None, 0, yaw)
    R = thrust_attitude(acc, yaw)
    assert np.allclose(R @ R.T, np.eye(3)) and np.allclose(R[:, 2], acc / np.linalg.norm(acc))
    assert np.allclose(pos + R @ B, pivot)                         # centre + R b = pivot
    if tilt_deg == 0.0:
        assert np.allclose(pos, pivot + [0, 0, 0.04])


def test_zero_offset_reference_is_unchanged():
    pivot = np.array([0.5, 0.1, 0.6])
    out = _kin_solver(pivot, np.array([1.0, 0.0, G]), np.zeros(3)).drone_kinematics(None, 0, 0.3)
    assert (out[0] == pivot).all()


class _Log:
    def info(self, m):
        pass

    warn = info


def _creep(pivots, rho, offset, target_deg=45.0, publish=None):
    c = CreepController(len(rho), rho, L, 20, 0.1, G, target_deg, 10.0,
                        lambda i: pivots[i], publish, _Log(),
                        pivot_offset=offset, drone_yaw=lambda i: 0.3 * i)
    return c


def test_creep_targets_the_centre_on_an_arc_about_the_attach_point():
    load, rho, pivots, centres = _ring(elev_deg=40.0)
    refs = {}
    c = _creep(pivots, rho, B, target_deg=50.0,
               publish=lambda i, nodes: refs.__setitem__(i, nodes))
    c.airborne_start = True
    c.step(load, pivots, [(1.0, L)] * 4, takeoff_seen=True)
    for i in range(4):
        attach = load[0:3] + rho[i]
        pts = [np.asarray(p) for p, _v, _a, _c in refs[i]]
        # the tracker target is the centre, 4 cm above the pivot on a level drone ...
        assert all(np.linalg.norm(p - [0, 0, 0.04] - attach) == pytest.approx(L) for p in pts)
        # ... and the first node starts next to where the centre is now
        assert np.linalg.norm(pts[0] - centres[i]) < 0.02


def test_ground_creep_first_reference_is_the_centre_plus_the_lead():
    load, rho, pivots, centres = _ring(elev_deg=30.0)
    refs = {}
    c = _creep(pivots, rho, B, publish=lambda i, nodes: refs.__setitem__(i, nodes[0][0]))
    c.step(load, pivots, [(1.0, L)] * 4, takeoff_seen=False)
    for i in range(4):
        assert np.allclose(refs[i], centres[i] + [0, 0, CREEP_LEAD])


@pytest.mark.parametrize('airborne', [False, True])
def test_zero_offset_creep_is_unchanged(airborne):
    load, rho, pivots, _ = _ring(elev_deg=30.0)
    out = []
    for kw in ({}, {'pivot_offset': np.zeros(3), 'drone_yaw': lambda i: 1.0}):
        refs = {}
        c = CreepController(4, rho, L, 20, 0.1, G, 45.0, 10.0, lambda i: pivots[i],
                            lambda i, nodes: refs.__setitem__(i, nodes), _Log(), **kw)
        c.airborne_start = airborne
        for _ in range(3):
            c.step(load, pivots, [(1.0, L)] * 4, takeoff_seen=True)
        out.append(refs)
    for i in range(4):
        for (p0, v0, a0, c0), (p1, v1, a1, c1) in zip(out[0][i], out[1][i]):
            assert (np.asarray(p0) == np.asarray(p1)).all()
            assert (np.asarray(v0) == np.asarray(v1)).all()


def test_zero_offset_pivot_is_the_measured_centre():
    load, rho, pivots, centres = _ring()
    p = _node(load, rho, centres, np.zeros(3), slot2drone=[1, 0, 3, 2])
    for i in range(4):
        assert p._pivot_at(i) is p._drone_at(i)


def test_centre_from_pivot_zero_offset_is_identity():
    v = np.array([1.0, 2.0, 3.0])
    assert (centre_from_pivot(v, [0, 0, 9.81], 0.4, [0, 0, 0]) == v).all()
