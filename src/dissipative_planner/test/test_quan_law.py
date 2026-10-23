"""Quan et al.'s per-robot law as printed (quan_law.py; card 2026-10-23_quan_exact): Eq. 3's node, its rate (R1),
the hand-over's force-free rest, the springs' and friction's signs, Eq. 10 as read (R6) and the velocity command (R7)."""
import math

import numpy as np
import pytest

from dissipative_planner.quan_law import QuanLaw, QuanParams, hor, k_d, node, node_rate

L, ELEV, Z_LOAD, M_Q, M_L = 1.0, math.radians(60.0), 1.0, 0.55, 0.86
H_C = Z_LOAD + 1.69


def _cone(n=4, yaw=0.0):
    """n quads on 1 m cables at 60 deg around a load at (0, 0, 1): the hand-over's settled hover."""
    load = np.array([0.0, 0.0, Z_LOAD])
    x = np.array([load + L * np.array([math.cos(ELEV) * math.cos(a), math.cos(ELEV) * math.sin(a), math.sin(ELEV)])
                  for a in yaw + 2.0 * math.pi * np.arange(n) / n])
    return x, load


def _law(x, load, xd=None, **kw):
    law = QuanLaw(len(x), M_Q, M_L, QuanParams(**kw))
    law.start(x, np.zeros_like(x) if xd is None else xd, load, load, H_C)
    return law


def test_eq3_puts_the_node_on_the_load_robot_line_at_the_design_altitude():
    x, load = _cone()
    for xi in x:
        q = node(xi, load, H_C)
        assert q[2] == pytest.approx(H_C)
        assert np.linalg.norm(np.cross(q - load, xi - load)) == pytest.approx(0.0, abs=1e-12)   # collinear
        assert float((q - load) @ (xi - load)) > 0.0                                           # beyond the robot
    assert np.allclose(node(x[0], load, x[0][2]), x[0])                                        # h_c = h_i: q = x
    # their code's node plane: 1.69 cable lengths over the load puts the node 0.82 m above a 60 deg quad
    assert np.linalg.norm(node(x[0], load, H_C) - x[0]) == pytest.approx((1.69 - math.sin(ELEV)) / math.sin(ELEV))


def test_the_node_rate_is_eq3_differentiated():
    rng = np.random.default_rng(3)
    x, load = _cone()
    for _ in range(5):
        xi, xL = x[1] + 0.05 * rng.standard_normal(3), load + 0.05 * rng.standard_normal(3)
        vi, vL, hcd = rng.standard_normal(3), rng.standard_normal(3), 0.3 * rng.standard_normal()
        h = 1e-6
        fd = (node(xi + h * vi, xL + h * vL, H_C + h * hcd) - node(xi - h * vi, xL - h * vL, H_C - h * hcd)) / (2 * h)
        assert np.allclose(node_rate(xi, vi, xL, vL, H_C, hcd), fd, atol=1e-6)
        assert node_rate(xi, vi, xL, vL, H_C, hcd)[2] == pytest.approx(hcd)


def test_k_d_is_the_robots_height_over_the_node_planes():
    assert k_d(Z_LOAD + 0.866, Z_LOAD, H_C) == pytest.approx(0.866 / 1.69)


def test_the_hand_over_is_force_free_whatever_the_formation():
    """l_ij(0), l_i(0) are taken at the hand-over, so any formation it hands over is the law's rest."""
    rng = np.random.default_rng(7)
    for n in (3, 4, 5):
        x, load = _cone(n, yaw=0.3)
        x = x + 0.05 * rng.standard_normal(x.shape)
        law = _law(x, load)
        for i in range(n):
            h1, h2 = law.forces(i, x, np.zeros_like(x), load, np.zeros(3), load, np.zeros(2), H_C)
            assert np.allclose(h1, 0.0, atol=1e-12) and np.allclose(h2, 0.0, atol=1e-12)


def test_a_formation_moving_with_the_path_feels_no_damping():
    x, load = _cone()
    law = _law(x, load)
    v = np.array([0.4, -0.1, 0.0])
    xd = np.tile(v, (4, 1))
    for i in range(4):
        h1, h2 = law.forces(i, x, xd, load, v, load, v[:2], H_C)
        assert np.allclose(h1 + h2, 0.0, atol=1e-12)


def test_the_springs_and_the_friction_restore():
    x, load = _cone()
    law = _law(x, load)
    zero = np.zeros_like(x)
    # the whole formation and its load 5 cm behind the path point in x: every robot pulled +x, the on-axis ones most
    sh = np.array([-0.05, 0.0, 0.0])
    f = [sum(law.forces(i, x + sh, zero, load + sh, np.zeros(3), load, np.zeros(2), H_C)) for i in range(4)]
    assert all(fi[0] > -1e-12 for fi in f) and sum(fi[0] for fi in f) > 0.0
    # the formation and load 5 cm low: the robot-node springs pull every robot up
    dn = np.array([0.0, 0.0, -0.05])
    assert all(sum(law.forces(i, x + dn, zero, load + dn, np.zeros(3), load, np.zeros(2), H_C))[2] > 0.0
               for i in range(4))
    # robots moving +y faster than the path: Eq. 9's friction opposes it, horizontally only
    xd = np.tile([0.0, 0.3, 0.2], (4, 1))
    h2 = law.forces(0, x, xd, load, np.array([0.0, 0.3, 0.2]), load, np.zeros(2), H_C)[1]
    assert h2[1] < 0.0 and h2[2] == 0.0
    assert np.allclose(hor(np.array([1.0, 2.0, 3.0])), [1.0, 2.0, 0.0])


def test_k_d_scales_the_node_terms_and_not_the_robot_node_spring():
    """R2: Eq. 7 as typeset. With only k_i on, the force does not depend on where h_c puts k_d; with only k_ij on,
    it is k_d times the node force."""
    x, load = _cone()
    dn = np.array([0.0, 0.0, -0.05])
    zero = np.zeros_like(x)
    only_ki = _law(x, load, k_ij=0.0, c_ij=0.0, c_i=0.0, f_c=0.0)
    h1 = only_ki.forces(0, x + dn, zero, load + dn, np.zeros(3), load, np.zeros(2), H_C)[0]
    e = node(x[0] + dn, load + dn, H_C) - (x[0] + dn)
    assert np.allclose(h1, only_ki.p.k_i * (1.0 - only_ki.li0[0] / np.linalg.norm(e)) * e)
    sh = np.array([-0.05, 0.0, 0.0])
    only_kij = _law(x, load, k_i=0.0, c_ij=0.0, c_i=0.0, f_c=0.0)
    h1 = only_kij.forces(0, x + sh, zero, load + sh, np.zeros(3), load, np.zeros(2), H_C)[0]
    q = [node(xi + sh, load + sh, H_C) for xi in x]
    q0 = np.array([0.0, 0.0, H_C])
    spring = sum((1.0 - l0 / np.linalg.norm(q[0] - qj)) * (q[0] - qj)
                 for qj, l0 in [(q0, only_kij.l0[0, 0])] + [(q[j], only_kij.l0[0, j + 1]) for j in (1, 2, 3)])
    assert np.allclose(h1, -k_d(x[0][2], load[2], H_C) * only_kij.p.k_ij * hor(spring))


def test_eq10_as_read_the_pull_and_the_weight_cancel_and_the_thrust_carries_them():
    """R6: a_cmd = a_expected + (h1 + h2)/m; the specific-thrust feedforward is g + a_cmd minus the modelled pull,
    and the four modelled pulls hold up the load."""
    x, load = _cone()
    law = _law(x, load)
    a_exp = np.array([0.3, 0.0, 0.0])
    pulls = []
    for i in range(4):
        v_d, a_ff, a_cable, a_cmd = law.command(i, x, np.zeros_like(x), load, np.zeros(3), load, np.zeros(2), a_exp,
                                                H_C, 0.1)
        assert np.allclose(a_cmd, a_exp)                                # force-free: the path's acceleration alone
        assert np.allclose(a_ff, [0.0, 0.0, 9.81] + a_exp - a_cable)
        assert np.allclose(v_d, 0.1 * a_exp)
        pulls.append(M_Q * a_cable)
    assert sum(p[2] for p in pulls) == pytest.approx(-M_L * 9.81)       # the cables pull down by the load's weight
    for i, p in enumerate(pulls):                                       # each toward the load
        u = (x[i] - load) / np.linalg.norm(x[i] - load)
        assert np.allclose(np.cross(p, u), 0.0, atol=1e-12) and float(p @ u) < 0.0
    assert np.allclose(law.cable_pull(0, x, load, taut_gate=0.0), 0.0)


def test_r7_integrates_the_velocity_command_and_r7b_does_not():
    """R7 (vcmd integrated): v_d accumulates a_cmd from the hand-over, bounded; R7b (measured, the default): v_d is the
    measured velocity plus one tick of a_cmd, whatever came before."""
    x, load = _cone()
    xd = np.tile([0.1, 0.0, 0.0], (4, 1))
    law = _law(x, load, xd=xd, v_max=0.5, vcmd='integrated')
    assert np.allclose(law.v_d, xd)
    for _ in range(50):
        v_d = law.command(0, x, xd, load, xd[0], load, xd[0][:2], np.array([1.0, 0.0, 0.0]), H_C, 0.1)[0]
    assert np.linalg.norm(v_d) == pytest.approx(0.5)
    law = _law(x, load, xd=xd)
    for _ in range(50):
        v_d = law.command(0, x, xd, load, xd[0], load, xd[0][:2], np.array([1.0, 0.0, 0.0]), H_C, 0.1)[0]
    assert np.allclose(v_d, [0.2, 0.0, 0.0])


def test_dynamic_nodes_start_on_eq3_rest_there_and_stay_on_the_node_plane():
    """R1 dynamic (S2/S4): the nodes start at Eq. 3's nodes, a still formation leaves them (and the force) at rest, and
    a quad pulled off its node drags the node toward it on the plane h_c."""
    x, load = _cone()
    law = _law(x, load, nodes='dynamic')
    zero = np.zeros_like(x)
    assert np.allclose(law.q, [node(xi, load, H_C) for xi in x])
    law.advance_nodes(x, zero, load, np.zeros(2), H_C, 0.1)
    assert np.allclose(law.q, [node(xi, load, H_C) for xi in x]) and np.allclose(law.qd, 0.0)
    for i in range(4):
        assert np.allclose(sum(law.forces(i, x, zero, load, np.zeros(3), load, np.zeros(2), H_C)), 0.0, atol=1e-12)
    moved = x.copy()
    moved[0] += [0.0, 0.0, -0.1]                       # quad 0 drops: its spring to the node stretches
    q0 = law.q[0].copy()
    for _ in range(5):
        law.advance_nodes(moved, zero, load, np.zeros(2), H_C, 0.1)
    assert np.allclose(law.q[:, 2], H_C) and np.linalg.norm(law.q[0] - q0) > 1e-4
    assert float((law.q[0] - q0) @ (moved[0] - q0)) > 0.0       # toward the quad, horizontally


def test_the_pull_model_carries_the_rods():
    """R6: each quad's modelled pull carries its share of the ring and its own rod."""
    x, load = _cone()
    law = _law(x, load, rod_mass=0.075)
    assert sum(law.cable_pull(i, x, load)[2] for i in range(4)) == pytest.approx(-(M_L + 4 * 0.075) * 9.81)


def test_only_robots_in_range_count():
    """Eq. 5: w_ij = 1 within r; the modelled pull shares the load over the robots perceived."""
    x, load = _cone()
    law = _law(x, load, range_m=1.0)                    # neighbours 0.71 m apart, opposite ones 1.0 m: not seen
    assert law.neighbours(0, x) == [1, 3]
    assert law.cable_pull(0, x, load)[2] == pytest.approx(-M_L * 9.81 / 3.0)
    assert _law(x, load).neighbours(0, x) == [1, 2, 3]


def _pull0(x, load):
    """Our OCP's cable term before the hand-over: the ring's share along each rod, per kg of drone (m/s^2)."""
    return [-(M_L / 4) * 9.81 / (xi - load)[2] * (xi - load) / M_Q for xi in x]


def test_r6b_is_off_by_default_and_names_its_readings():
    assert QuanParams().fz == 'model'
    with pytest.raises(ValueError):
        QuanParams(fz='true')
    x, load = _cone()
    law = QuanLaw(4, M_Q, M_L, QuanParams(rod_mass=0.075))
    law.start(x, np.zeros_like(x), load, load, H_C, a_cable0=_pull0(x, load))
    assert np.allclose(law.b_z, 0.0)                                    # R6 alone ignores the drone's earlier thrust
    _, a_ff, a_cable, a_cmd = law.command(0, x, np.tile([0.0, 0.0, 0.3], (4, 1)), load, np.zeros(3), load,
                                          np.zeros(2), np.zeros(3), H_C, 0.1)
    assert np.allclose(law.b_z, 0.0) and a_ff[2] == pytest.approx(9.81 + a_cmd[2] - a_cable[2])


def test_r6b_starts_on_the_thrust_flown_before_the_hand_over():
    """R6b: b_i(0) makes the first vertical feed-forward the OCP's, whatever the pull model adds (here the rod the
    twin's drone_mass already holds); the horizontal stays the model's."""
    x, load = _cone()
    law = QuanLaw(4, M_Q, M_L, QuanParams(rod_mass=0.075, fz='autopilot'))
    a0 = _pull0(x, load)
    law.start(x, np.zeros_like(x), load, load, H_C, a_cable0=a0)
    assert np.allclose(law.b_z, -0.075 * 9.81 / M_Q)
    for i in range(4):
        a_ff, a_cable = law.command(i, x, np.zeros_like(x), load, np.zeros(3), load, np.zeros(2), np.zeros(3),
                                    H_C, 0.1)[1:3]
        assert a_ff[2] == pytest.approx(9.81 - a0[i][2])
        assert np.allclose(a_ff[:2], -a_cable[:2])
    law.start(x, np.zeros_like(x), load, load, H_C, a_cable0=[None, np.full(3, np.nan), a0[2], a0[3]])
    assert law.b_z[0] == 0.0 and law.b_z[1] == 0.0 and law.b_z[2] < 0.0


def test_r6b_integrates_the_vertical_velocity_the_drone_did_not_follow():
    """R6b: a robot pushed down by its node spring that does not descend lowers its thrust tick by tick, within
    fz_max; one that follows its command leaves the integral alone."""
    x, load = _cone()
    law = QuanLaw(4, M_Q, M_L, QuanParams(k_i=1.0, c_i=0.0, fz='autopilot', fz_ki=2.0, fz_max=0.5))
    law.start(x, np.zeros_like(x), load, load, H_C)
    up = x.copy()
    up[:, 2] += 0.2                                       # the formation 0.2 m high: each spring pushes down
    zero = np.zeros_like(x)
    a_cmd = law.command(0, up, zero, load + [0, 0, 0.2], np.zeros(3), load, np.zeros(2), np.zeros(3), H_C, 0.1)[3]
    assert a_cmd[2] < 0.0 and law.b_z[0] == 0.0           # the first tick sets the command
    b = []
    for _ in range(3):
        law.command(0, up, zero, load + [0, 0, 0.2], np.zeros(3), load, np.zeros(2), np.zeros(3), H_C, 0.1)
        b.append(law.b_z[0])
    assert b[0] == pytest.approx(2.0 * a_cmd[2] * 0.1 * 0.1) and b[2] < b[1] < b[0] < 0.0
    for _ in range(500):
        law.command(0, up, zero, load + [0, 0, 0.2], np.zeros(3), load, np.zeros(2), np.zeros(3), H_C, 0.1)
    assert law.b_z[0] == pytest.approx(-0.5)
    law.start(x, np.zeros_like(x), load, load, H_C)
    for _ in range(5):                                    # followed: v_z is what was commanded
        v_d = law.command(1, x, np.tile(law.v_d[1], (4, 1)), load, np.zeros(3), load, np.zeros(2), np.zeros(3),
                          H_C, 0.1)[0]
    assert law.b_z[1] == pytest.approx(0.0, abs=1e-12)
