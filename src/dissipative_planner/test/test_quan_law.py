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
    law = _law(x, load, xd=xd, vcmd='feedforward')        # R7c: a_cmd in the feed-forward alone
    v_d, a_ff, a_cable, a_cmd = law.command(0, x, xd, load, xd[0], load, xd[0][:2], np.array([1.0, 0.0, 0.0]), H_C,
                                            0.1)
    assert np.allclose(v_d, xd[0]) and a_cmd[0] == pytest.approx(1.0) and np.allclose(a_ff[:2], (a_cmd - a_cable)[:2])


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


def test_the_pull_split_counts_the_rod_horizontally_only():
    """R6c: vertically each quad's share of the ring (drone_mass holds its rod); horizontally that share and the
    rod's load-end share, g cot phi, along the rod's horizontal direction toward the load."""
    x, load = _cone()
    law = _law(x, load, rod_mass=0.055)
    for i in range(4):
        f = law.cable_pull(i, x, load)
        u = (x[i] - load) / np.linalg.norm(x[i] - load)
        assert f[2] == pytest.approx(-(M_L / 4) * 9.81)
        assert np.linalg.norm(f[:2]) == pytest.approx((M_L / 4 + 0.055) * 9.81 / math.tan(ELEV))
        assert float(f[:2] @ u[:2]) < 0.0 and abs(float(np.cross(f[:2], u[:2]))) < 1e-12
    assert np.allclose(_law(x, load).cable_pull(1, x, load), -(M_L / 4) * 9.81 / math.sin(ELEV)
                       * (x[1] - load) / np.linalg.norm(x[1] - load))    # rod 0: the share along the rod
    assert np.allclose(law.cable_pull(0, x, load, taut_gate=0.0), 0.0)


def test_only_robots_in_range_count():
    """Eq. 5: w_ij = 1 within r; the modelled pull shares the load over the robots perceived."""
    x, load = _cone()
    law = _law(x, load, range_m=1.0)                    # neighbours 0.71 m apart, opposite ones 1.0 m: not seen
    assert law.neighbours(0, x) == [1, 3]
    assert law.cable_pull(0, x, load)[2] == pytest.approx(-M_L * 9.81 / 3.0)
    assert _law(x, load).neighbours(0, x) == [1, 2, 3]


def test_a_expected_is_the_paths_unless_zero():
    """R5: a_cmd carries the path's acceleration ('path', the default); 'zero' drops it, nothing else changes."""
    x, load = _cone()
    a = np.array([0.3, -0.1, 0.0])
    args = (x, np.zeros_like(x), load, np.zeros(3), load, np.zeros(2), a, H_C, 0.1)
    assert QuanParams().aexp == 'path'
    with pytest.raises(ValueError):
        QuanParams(aexp='half')
    a_on = _law(x, load).command(0, *args)[3]
    a_off = _law(x, load, aexp='zero').command(0, *args)[3]
    assert np.allclose(a_on - a_off, a) and np.allclose(a_off, 0.0)


def test_the_dynamic_centre_follows_the_path_point_and_feels_the_nodes():
    """R3 dynamic (S2 Eq. 16): the centre starts on the path point at rest, is driven toward a moved path point, and
    the robots' springs see it, not the path point; 'path' (the default) leaves no state."""
    x, load = _cone()
    assert QuanParams().centre == 'path'
    law = _law(x, load, centre='dynamic', k_ij=0.5, c_ij=0.25, f_c=0.5)
    assert np.allclose(law.x0, load[:2]) and np.allclose(law.xd0, 0.0)
    zero = np.zeros_like(x)
    p_d = load + [0.5, 0.0, 0.0]
    h1 = law.forces(0, x, zero, load, np.zeros(3), p_d, np.zeros(2), H_C)[0]
    assert np.allclose(h1, 0.0, atol=1e-12)                 # the centre has not moved yet: the hand-over's rest
    for _ in range(20):
        law.advance_nodes(x, zero, p_d, np.zeros(2), H_C, 0.1, x_L=load, xd_L=np.zeros(3))
    assert 0.0 < law.x0[0] < 0.5 and law.xd0[0] > 0.0 and abs(law.x0[1]) < 1e-12
    h1 = law.forces(0, x, zero, load, np.zeros(3), p_d, np.zeros(2), H_C)[0]
    assert h1[0] > 0.0                                       # the springs pull the formation after its centre
    free = _law(x, load, centre='dynamic', k_ij=0.0, c_ij=0.0, f_c=0.0)
    for _ in range(10):
        free.advance_nodes(x, zero, p_d, np.zeros(2), H_C, 0.1, x_L=load, xd_L=np.zeros(3))
    pulled = _law(x, load, centre='dynamic', k_ij=2.0, c_ij=0.0, f_c=0.0)
    for _ in range(10):
        pulled.advance_nodes(x, zero, p_d, np.zeros(2), H_C, 0.1, x_L=load, xd_L=np.zeros(3))
    assert pulled.x0[0] < free.x0[0]                         # the nodes hold it back (Eq. 16's reactions)
    path = _law(x, load)
    path.advance_nodes(x, zero, p_d, np.zeros(2), H_C, 0.1, x_L=load, xd_L=np.zeros(3))
    assert np.allclose(path.x0, load[:2])
