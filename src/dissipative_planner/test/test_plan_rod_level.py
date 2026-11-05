"""plan_rod_level (twin only, off by default; card 2026-10-31_rod_in_plan): with rod_mass in the plan, the references
balance the rod. Tensions from lift and the level moments alone, the cables leaning to take the sideways force no
tension can. Off, nothing changes bit for bit. An even layout keeps the equal split and its directions. The resize and
the join are the real resize_fleet and _attach_ocp."""
from types import MethodType, SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('acados_template')
from mpc_planner.geometry import (  # noqa: E402
    attach_points, balanced_tensions, level_rod_refs, nominal_cable_dirs, rod_neutral_tension)
from mpc_planner.load_cable_dynamics import CABLE_DIM, LOAD_DIM, LoadCableDynamics  # noqa: E402
from mpc_planner.planner_node import LoadPlanner  # noqa: E402
from mpc_planner.reference_builder import ReferenceBuilder, blend_refs, rebalance_incumbents, unload_tensions  # noqa: E402
from test_partner_contact import _fold_in_node  # noqa: E402
from test_soft_detach import _detach, _node as _soft_node  # noqa: E402

M, R, H, L, ELEV = 0.86, 0.225, 0.02, 1.0, 60.0
M_R, LAM = 0.075, 0.27
J = [0.02733, 0.02733, 0.05452]
T_AZ = [0.0, 180.0, 270.0]      # the T: the pair on the x axis, the middle carrier at -y
EVEN = [[0.0, 180.0], [30.0, 150.0, 270.0], [0.0, 90.0, 180.0, 270.0], [45.0 * k for k in range(8)]]
UNEVEN = [T_AZ, [30.0, 90.0, 150.0, 270.0], [45.0 * k for k in range(7)], [0.0, 90.0, 180.0, 270.0, 315.0]]
G_ = np.array([0.0, 0.0, -9.81])


def _dirs(az, r=R):
    rho = attach_points(len(az), r, H, list(az))
    return rho, nominal_cable_dirs(rho, ELEV)


def _dyn(az, m=M, r=R):
    return LoadCableDynamics(len(az), m, J, [L] * len(az), attach_points(len(az), r, H, list(az)), 0.55,
                             rod_mass=M_R, rod_lam=LAM)


def _accels(dyn, s, t):
    """The model's load accelerations with the ring level and at rest, on these directions and tensions."""
    x = np.zeros(dyn.nx)
    x[6] = 1.0
    for i in range(dyn.n):
        b = LOAD_DIM + CABLE_DIM * i
        x[b:b + 3], x[b + 12] = s[i], t[i]
    xdot = np.asarray(dyn.load_cable_dynamics()(x, np.zeros(dyn.nu), dyn.geom_values())).ravel()
    return xdot[3:6], xdot[10:13]


def _old_balanced_tensions(rho, s_dirs, load_mass, g=9.81, t_min=0.1, rod_mass=0.0, rod_lam=0.0):
    """balanced_tensions as it was before the switch, to hold the off path bit for bit."""
    n = len(rho)
    s = [np.asarray(v, float).reshape(3) for v in s_dirs]
    r = [np.asarray(v, float).reshape(3) for v in rho]
    A = np.zeros((6, n))
    for i in range(n):
        A[0:3, i] = s[i]
        A[3:6, i] = np.cross(r[i], s[i])
    b = np.array([0.0, 0.0, -float(load_mass) * g, 0.0, 0.0, 0.0])
    if rod_mass > 0.0:
        G = np.array([0.0, 0.0, -g])
        for i in range(n):
            R_i = rod_mass * (rod_lam * float(G @ s[i]) * s[i] + (1.0 - rod_lam) * G)
            b[0:3] += R_i
            b[3:6] += np.cross(r[i], R_i)
    sz = -np.mean([v[2] for v in s])
    t_eq = np.full(n, float(load_mass) * g / max(n * sz, 1e-9))
    t = t_eq + np.linalg.pinv(A) @ (b - A @ t_eq)
    return [float(max(v, t_min)) for v in t]


@pytest.mark.parametrize('az', EVEN + UNEVEN)
@pytest.mark.parametrize('rod', [(0.0, 0.0), (M_R, LAM)])
def test_off_balanced_tensions_is_the_old_one_bit_for_bit(az, rod):
    rho, s = _dirs(az)
    assert balanced_tensions(rho, s, M, rod_mass=rod[0], rod_lam=rod[1]) == _old_balanced_tensions(
        rho, s, M, rod_mass=rod[0], rod_lam=rod[1])


def test_off_the_builder_keeps_the_nominal_directions_and_the_code_shares():
    dyn = _dyn(T_AZ)
    s = nominal_cable_dirs(dyn.rho, ELEV)
    b = ReferenceBuilder(dyn, 3, s, 0.1, None)
    assert b._s_nom is s and not b.rod_level
    assert b._t_nom == _old_balanced_tensions(dyn.rho, s, M, rod_mass=M_R, rod_lam=LAM)
    assert b._t_nom[2] == pytest.approx(0.2506, abs=1e-3)          # R3000's reference


def test_off_the_rebalance_is_as_before():
    rho, s = _dirs([0.0, 120.0, 240.0, 300.0])
    t = [1.0, 2.0, 3.0, 0.5]
    assert rebalance_incumbents(rho, s, t, M) == rebalance_incumbents(rho, s, t, M, rod_mass=0.0, rod_lam=LAM)


@pytest.mark.parametrize('az', EVEN)
def test_an_even_layout_keeps_its_equal_split_and_directions(az):
    rho, s = _dirs(az)
    off = balanced_tensions(rho, s, M, rod_mass=M_R, rod_lam=LAM)
    assert np.allclose(balanced_tensions(rho, s, M, rod_mass=M_R, rod_lam=LAM, level=True), off, rtol=1e-9)
    s2, t2 = level_rod_refs(rho, s, M, M_R, LAM)
    assert all(np.array_equal(a, b) for a, b in zip(s, s2))
    assert np.allclose(t2, off, rtol=1e-9) and np.allclose(t2, t2[0], rtol=1e-9)


def test_the_t_gets_the_level_share_the_code_path_does_not():
    rho, s = _dirs(T_AZ)
    assert np.allclose(balanced_tensions(rho, s, M, rod_mass=M_R, rod_lam=LAM), [5.934, 5.934, 0.251], atol=2e-3)
    assert np.allclose(balanced_tensions(rho, s, M, rod_mass=M_R, rod_lam=LAM, level=True),
                       [5.646, 5.646, 0.826], atol=2e-3)


def test_the_t_references_lean_the_pair_and_hardly_the_middle():
    rho, s = _dirs(T_AZ)
    s2, t2 = level_rod_refs(rho, s, M, M_R, LAM)
    assert np.allclose(t2, [5.665, 5.665, 0.791], atol=2e-3)
    lean = [np.degrees(np.arccos(np.clip(a @ b, -1.0, 1.0))) for a, b in zip(s, s2)]
    assert lean[0] == pytest.approx(lean[1]) and 1.4 < lean[0] < 1.8 and lean[2] < 0.3
    assert s2[0][1] < 0.0 and s2[1][1] < 0.0                        # drone->load: the pair's drones lean to +y
    assert all(np.linalg.norm(v) == pytest.approx(1.0) for v in s2)


@pytest.mark.parametrize('az', UNEVEN)
def test_the_references_are_an_equilibrium_of_the_planner_model(az):
    r = 0.5 if len(az) > 4 else R
    m = 0.9 if len(az) > 4 else M
    dyn = _dyn(az, m=m, r=r)
    s = nominal_cable_dirs(dyn.rho, ELEV)
    s2, t2 = level_rod_refs(dyn.rho, s, m, M_R, LAM)
    a, w = _accels(dyn, s2, t2)
    assert np.allclose(a, 0.0, atol=1e-6) and np.allclose(w[:2], 0.0, atol=1e-6)
    assert min(t2) > 0.5
    a0, w0 = _accels(dyn, s, balanced_tensions(dyn.rho, s, m, rod_mass=M_R, rod_lam=LAM))
    assert np.abs(np.r_[a0, w0[:2]]).max() > 0.01                   # the code path's references are not


def test_the_middle_share_is_what_holds_its_rod_up():
    """Level, the plate feels no vertical force. The share is the rod's own, its sideways part what the pair takes."""
    rho, s = _dirs(T_AZ)
    s2, t2 = level_rod_refs(rho, s, M, M_R, LAM)
    assert t2[2] == pytest.approx(rod_neutral_tension(s2[2], M_R, LAM), abs=0.02)
    assert rod_neutral_tension(np.array([0.0, 0.0, -1.0]), M_R, LAM) == pytest.approx(M_R * 9.81)
    F = -t2[2] * s2[2] + M_R * (LAM * float(G_ @ s2[2]) * s2[2] + (1.0 - LAM) * G_)
    assert abs(F[2]) < 0.02 and F[1] < -0.25                        # outward at plate 3, w cot e


def test_the_rebalance_counts_the_rods():
    rho, s = _dirs([30.0, 90.0, 150.0, 270.0])
    t = rebalance_incumbents(rho, s, [3.0, 2.0, 3.0, 4.2], M, rod_mass=M_R, rod_lam=LAM)
    F = np.zeros(3)
    Mo = np.zeros(3)
    for ri, si, ti in zip(rho, s, t):
        f = -ti * si + M_R * (LAM * float(G_ @ si) * si + (1.0 - LAM) * G_)
        F, Mo = F + f, Mo + np.cross(ri, f)
    assert t[3] == 4.2 and min(t) > 0.5
    assert abs(F[2] + M * G_[2]) < 1e-6 and np.allclose(Mo[:2], 0.0, atol=1e-6)


class _Stub:
    N, dt = 20, 0.1

    def __init__(self, dyn):
        self.dyn, self.last_X = dyn, np.zeros((3, 3))

    def set_geometry(self, rho=None, cable_lengths=None):
        if rho is not None:
            self.dyn.rho = [np.asarray(r, float) for r in rho]


def _log():
    return SimpleNamespace(warn=lambda *_: None, info=lambda *_: None, error=lambda *_: None)


def _planner(az_now, sizes, on):
    p = SimpleNamespace(
        _solvers={}, n=len(az_now), cable_len=L, cable_len_i=[L] * len(az_now), cable_elev_deg=ELEV, dt=0.1, traj=None,
        psi0=0.0, _yaw_datum_latched=False, get_logger=_log, slot2drone=list(range(len(az_now))), _rod_level=on)
    for n in sizes:
        d = LoadCableDynamics(n, M, J, [L] * n, attach_points(n, R, H), 0.55, rod_mass=M_R, rod_lam=LAM)
        p._solvers[n] = (d, _Stub(d), attach_points(n, R, H))
    return p


def test_a_resize_with_it_on_flies_the_level_references():
    p = _planner([0.0, 90.0, 180.0, 270.0], [3, 4], on=True)
    rho = attach_points(3, R, H, T_AZ)
    assert LoadPlanner.resize_fleet(p, 3, [0, 2, 3], rho=rho, cable_lengths=[L] * 3)
    assert p.refs.rod_level and np.allclose(p.refs._t_nom, [5.665, 5.665, 0.791], atol=2e-3)
    assert not np.allclose(p.refs._s_nom[0], p._s_nom[0])          # the builder leans, the node's nominal stays radial
    assert np.allclose(p._s_nom[0], nominal_cable_dirs(rho, ELEV)[0])
    p.refs.update((0.0, 0.0), 1.0, 0.0, 1.0, 0.0, 0.0)
    s_k, t_k = p.refs._cable_refs(0)                                # what the OCP is given, no ring force learned
    a, w = _accels(p.dyn, s_k, t_k)
    assert np.allclose(a, 0.0, atol=1e-6) and np.allclose(w[:2], 0.0, atol=1e-6)
    assert LoadPlanner.resize_fleet(p, 4, [0, 1, 2, 3], rho=attach_points(4, R, H, [0.0, 90.0, 180.0, 270.0]),
                                    cable_lengths=[L] * 4)           # the fleet whole again: even, no lean
    assert np.allclose(p.refs._t_nom, p.refs._t_nom[0], rtol=1e-9)
    assert all(np.array_equal(a, b) for a, b in zip(p.refs._s_nom, p._s_nom))


def test_a_resize_with_it_off_is_the_code_path():
    p = _planner([0.0, 90.0, 180.0, 270.0], [3], on=False)
    rho = attach_points(3, R, H, T_AZ)
    assert LoadPlanner.resize_fleet(p, 3, [0, 2, 3], rho=rho, cable_lengths=[L] * 3)
    assert p.refs._s_nom is p._s_nom and not p.refs.rod_level
    assert p.refs._t_nom == _old_balanced_tensions(p.dyn.rho, p._s_nom, M, rod_mass=M_R, rod_lam=LAM)


def _joining(az_now, on, new_at=(0.0, R, H), n_new=None):
    """The real _attach_ocp on the real resize_fleet and a real builder: the fleet on az_now, QUAD4 (drone 3) caught."""
    f = _fold_in_node()
    n = len(az_now)
    n_new = n + 1 if n_new is None else n_new
    p = _planner(az_now, [n, n_new], on)
    f.__dict__.update(p.__dict__)
    f.rho = attach_points(n, R, H, list(az_now))
    f.dyn, f.solver, _ = f._solvers[n]
    f.solver.set_geometry(rho=f.rho)
    f.slot2drone = list(range(n))
    f._s_nom = nominal_cable_dirs(f.rho, ELEV)
    f.refs = ReferenceBuilder(f.dyn, n, f._s_nom, 0.1, None, rod_level=on)
    f.traj = SimpleNamespace(yaw_at=lambda t: (0.0, 0.0))
    f.traj_t = 0.0
    f.resize_fleet = MethodType(LoadPlanner.resize_fleet, f)
    f._prime_solver = lambda: None
    f.rejoin_rho = np.array(new_at, float)
    f.attach_pos[0] = f.rejoin_rho + np.array([0.0, 0.5, 1.0 + np.sin(np.radians(ELEV)) + 0.04])
    return f


def test_a_join_with_it_on_blends_from_the_lean_to_the_new_level_references():
    f = _joining(T_AZ, on=True)
    s_t = [v.copy() for v in f.refs._s_nom]
    t_t = list(f.refs._t_nom)
    assert f._attach_ocp(3, rejoin={'rho': f.rejoin_rho, 'len': L}) is True
    assert f.n == 4 and f.refs.rod_level
    # the square: even again, no lean, the shares with the rods in
    assert np.allclose(f.refs._t_nom, f.refs._t_nom[0], rtol=1e-6) and f.refs._t_nom[0] > 3.0
    s_from, t_from, T, _ = f.refs._blend
    assert all(np.allclose(a, b) for a, b in zip(s_from[:3], s_t))  # from the T's leaning directions, not radial ones
    assert t_from[:3] == t_t
    dvec = f.dyn.rho[-1] + np.array([0.0, 0.0, 1.0]) - f.attach_pos[0]
    s_new = dvec / np.linalg.norm(dvec)
    assert t_from[3] == pytest.approx(f._attach_t_start_new + rod_neutral_tension(s_new, M_R, LAM))
    f.refs._blend = (s_from, t_from, T, 0.5 * T)
    s_k, t_k = f.refs._refs_at(0)                                   # midway: incumbents rebalanced, the rods counted
    a, w = _accels(f.dyn, s_k, t_k)
    assert abs(a[2]) < 1e-6 and np.allclose(w[:2], 0.0, atol=1e-6)


def test_a_join_into_a_t_with_it_on_gets_the_t_references():
    f = _joining([0.0, 180.0], on=True, new_at=(0.0, -R, H))
    assert f._attach_ocp(3, rejoin={'rho': f.rejoin_rho, 'len': L}) is True
    assert f.n == 3 and np.allclose(f.refs._t_nom, [5.665, 5.665, 0.791], atol=2e-3)
    a, w = _accels(f.dyn, f.refs._s_nom, f.refs._t_nom)
    assert np.allclose(a, 0.0, atol=1e-6) and np.allclose(w[:2], 0.0, atol=1e-6)


def test_a_join_with_it_off_is_the_code_path():
    f = _joining([0.0, 180.0], on=False, new_at=(0.0, -R, H))
    assert f._attach_ocp(3, rejoin={'rho': f.rejoin_rho, 'len': L}) is True
    assert f.refs._s_nom is f._s_nom
    assert f.refs._t_nom == _old_balanced_tensions(f.dyn.rho, f._s_nom, M)   # no rod terms in this path, as before
    s_from, t_from, _, _ = f.refs._blend
    assert all(np.array_equal(a, b) for a, b in zip(s_from[:2], f._s_nom[:2]))
    assert t_from[2] == f._attach_t_start_new


def _old_unload_tensions(rho, s_dirs, load_mass, k, t_leave, rod_mass=0.0, rod_lam=0.0, g=9.81, t_min=0.1):
    """unload_tensions as it was before the switch, to hold the off path bit for bit."""
    n = len(rho)
    keep = [i for i in range(n) if i != k]
    s = [np.asarray(v, float).reshape(3) for v in s_dirs]
    r = [np.asarray(v, float).reshape(3) for v in rho]
    b = np.array([0.0, 0.0, -float(load_mass) * g, 0.0, 0.0, 0.0])
    if rod_mass > 0.0:
        G = np.array([0.0, 0.0, -g])
        for i in range(n):
            R_i = rod_mass * (rod_lam * float(G @ s[i]) * s[i] + (1.0 - rod_lam) * G)
            b[0:3] += R_i
            b[3:6] += np.cross(r[i], R_i)
    f_k = float(t_leave) * s[k]
    b = b - np.concatenate([f_k, np.cross(r[k], f_k)])
    A = np.stack([np.concatenate([s[i], np.cross(r[i], s[i])]) for i in keep], axis=1)
    sz = -np.mean([s[i][2] for i in keep])
    t_eq = np.full(len(keep), -b[2] / max(len(keep) * sz, 1e-9))
    t = t_eq + np.linalg.pinv(A) @ (b - A @ t_eq)
    out = [0.0] * n
    for i, v in zip(keep, t):
        out[i] = float(max(v, t_min))
    out[k] = float(t_leave)
    return out


@pytest.mark.parametrize('az', [[0.0, 90.0, 180.0, 270.0], [30.0, 90.0, 150.0, 270.0]])
@pytest.mark.parametrize('rod', [(0.0, 0.0), (M_R, LAM)])
def test_off_the_unload_target_is_the_old_one_bit_for_bit(az, rod):
    rho, s = _dirs(az)
    assert unload_tensions(rho, s, M, 1, 0.5, rod_mass=rod[0], rod_lam=rod[1]) == _old_unload_tensions(
        rho, s, M, 1, 0.5, rod_mass=rod[0], rod_lam=rod[1])


def _rod_soft(az, leaver_t):
    """test_soft_detach's node on a real rod model with the switch on; slot 1 leaves."""
    f = _soft_node(leaver_t=leaver_t)
    dyn = _dyn(az)
    f.__dict__.update(dyn=dyn, rho=list(dyn.rho), _rod_level=True, _unload_t=0.5)
    f.refs = ReferenceBuilder(dyn, len(az), nominal_cable_dirs(dyn.rho, ELEV), 0.1, None, rod_level=True)
    return f


def _plate_force(s, t):
    return -t * s + M_R * (LAM * float(G_ @ s) * s + (1.0 - LAM) * G_)


def test_soft_detach_unloads_the_leaver_to_its_rods_own_pull():
    f = _rod_soft([0.0, 90.0, 180.0, 270.0], leaver_t=0.8)
    _detach(f, 1)
    u = f._unload
    s = f.refs._s_nom
    assert u['leave'] == pytest.approx(rod_neutral_tension(s[1], M_R, LAM)) and u['target'][1] == u['leave']
    assert abs(_plate_force(s[1], u['target'][1])[2]) < 1e-9        # no vertical force left on its plate
    assert abs(_plate_force(s[1], 0.5)[2] + 0.25) < 0.01            # detach_unload_t 0.5 N hung it on the plate
    a, w = _accels(f.dyn, s, u['target'])                           # the square with its leaver unloaded is at rest
    assert np.allclose(a, 0.0, atol=1e-6) and np.allclose(w, 0.0, atol=1e-6)
    assert np.allclose(u['target'], [5.66, 0.79, 5.66, 0.79], atol=0.02)


def test_the_unload_gate_agrees_with_the_target():
    for t_planned, released in ((1.0, True), (1.2, False)):      # neutral 0.79 + 0.3: 0.5 + 0.3 would refuse the first
        f = _rod_soft([0.0, 90.0, 180.0, 270.0], leaver_t=t_planned)
        _detach(f, 1)
        f.refs._blend = None
        f._unload_tick()
        assert bool(f.calls) == released and (f._unload is None) == released


def test_soft_detach_off_reads_detach_unload_t_as_before():
    f = _soft_node(leaver_t=0.2)
    _detach(f, 1)
    assert f._unload['leave'] == f._unload_t == f._unload['target'][1]


def test_the_lean_steps_at_a_release_because_the_force_does():
    """Unloaded the square is at rest with no lean. The release takes the leaver's outward pull away, the T's lean answers
    it at once and the tensions hardly move: blending the lean in would plan a force that is already gone for a second."""
    f = _rod_soft([0.0, 90.0, 180.0, 270.0], leaver_t=0.8)
    _detach(f, 1)
    target, s_sq = f._unload['target'], f.refs._s_nom
    keep = [0, 2, 3]
    pull_gone = _plate_force(s_sq[1], target[1])[1]
    assert pull_gone == pytest.approx(0.31, abs=0.01)
    dyn = _dyn(T_AZ)
    b = ReferenceBuilder(dyn, 3, nominal_cable_dirs(dyn.rho, ELEV), 0.1, None, rod_level=True)
    assert np.allclose(b._t_nom, [target[i] for i in keep], atol=0.01)
    assert s_sq[0][1] == 0.0 and b._s_nom[0][1] == pytest.approx(-0.028, abs=0.001)
    a, _ = _accels(dyn, [s_sq[i] for i in keep], [target[i] for i in keep])
    assert a[1] == pytest.approx(-pull_gone / M, abs=0.01)           # in the model too, on the same tick as in the plant


@pytest.mark.parametrize('az,new,worst_on,worst_off', [(T_AZ, 90.0, 0.47, 0.41), ([0.0, 180.0], 270.0, 0.02, 0.06),
                                                       ([30.0, 150.0, 270.0], 90.0, 0.29, 0.28)])
def test_the_mid_blend_sideways_force_is_the_size_it_always_was(az, new, worst_on, worst_off):
    """A join's blend balances lift and the level moments only (rebalance_incumbents): the sideways row is left out, as
    without the switch. With the rods counted it stays the size the off path has flown."""
    def peak(on):
        n = len(az) + 1
        dyn = _dyn(list(az) + [new])
        s = nominal_cable_dirs(dyn.rho, ELEV)
        old = attach_points(n - 1, R, H, list(az))
        s_old = nominal_cable_dirs(old, ELEV)
        sf, tf = level_rod_refs(old, s_old, M, M_R, LAM) if on else (s_old, balanced_tensions(old, s_old, M))
        sn, tn = level_rod_refs(dyn.rho, s, M, M_R, LAM) if on else (s, balanced_tensions(dyn.rho, s, M))
        v = np.array([0.0, 0.0, -1.0])
        t0 = 0.1 + (rod_neutral_tension(v, M_R, LAM) if on else 0.0)
        worst = 0.0
        for a in np.linspace(0.0, 1.0, 21):
            s_b, t_b = blend_refs(list(sf) + [v], sn, list(tf) + [t0], tn, a)
            kw = dict(rod_mass=M_R, rod_lam=LAM) if on else {}
            t_b = rebalance_incumbents(dyn.rho, s_b, t_b, M, **kw)
            F = sum((-ti * si + (_plate_force(si, 0.0) if on else 0.0)) for ti, si in zip(t_b, s_b)) + M * G_
            assert abs(F[2]) < 1e-5
            worst = max(worst, float(np.hypot(F[0], F[1])))
        return worst
    assert peak(True) < worst_on and peak(False) < worst_off

