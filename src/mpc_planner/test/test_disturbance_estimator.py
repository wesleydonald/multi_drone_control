"""DisturbanceEstimator (card 2026-10-03_offset_free_mpc): a known load force and a known common
drone force are recovered from a consistent static balance, with or without massive rods; the
filter converges with its time constant, freezes when not gated and holds at its bound."""
import numpy as np
import pytest

from mpc_planner.disturbance_estimator import G, DisturbanceEstimator, rod_end_loads, solve_split

N, M_D, M_L, M_R, LAM = 3, 0.475, 0.86, 0.075, 0.267


def scene(d_L, d_D, m_r=0.0, lam=0.0, elev_deg=50.0, n=N, m_d=M_D):
    """Rod directions and the thrusts that hold every body still under d_L and d_D."""
    th = np.radians(elev_deg)
    s = [np.array([-np.cos(th) * np.cos(a), -np.cos(th) * np.sin(a), -np.sin(th)])
         for a in np.radians(np.arange(n) * 360.0 / n + 10.0)]
    loads = [rod_end_loads(si, m_r, lam) for si in s]
    # load: sum t_i (-s_i) = -m_L g - d_L - sum(on_load)  -> solve the tensions (n >= 3)
    need = -M_L * G - np.asarray(d_L, float) - sum(on_load for _, on_load in loads)
    t = np.linalg.lstsq(np.column_stack([-si for si in s]), need, rcond=None)[0]
    thrust = [-(m_d * G + t[i] * s[i] + np.asarray(d_D, float) + loads[i][0]) for i in range(n)]
    return s, thrust, t


@pytest.mark.parametrize('m_r,lam', [(0.0, 0.0), (M_R, LAM)])
def test_static_split_is_exact(m_r, lam):
    d_L, d_D = np.array([-0.5, 0.2, -0.3]), np.array([-0.15, 0.05, 0.1])
    s, thrust, t = scene(d_L, d_D, m_r, lam)
    zero = [np.zeros(3)] * N
    eL, eD, et, cond = solve_split(s, thrust, zero, np.zeros(3), M_D, M_L, m_r, lam)
    assert np.allclose(eL, d_L, atol=1e-9) and np.allclose(eD, d_D, atol=1e-9)
    assert np.allclose(et, t, atol=1e-9) and np.isfinite(cond)


def test_rod_weight_reaches_the_two_ends_in_full():
    s = np.array([-0.5, 0.0, -np.sqrt(0.75)])
    on_d, on_l = rod_end_loads(s, M_R, LAM)
    assert np.allclose(on_d + on_l, M_R * G)


def test_filter_converges_freezes_and_holds_its_bound():
    d_L, d_D = np.array([-0.5, 0.0, 0.0]), np.array([-0.15, 0.0, 0.0])
    s, thrust, _ = scene(d_L, d_D)
    v0 = [np.zeros(3)] * N
    est = DisturbanceEstimator(N, tau=0.6)
    for k in range(151):                       # 15 s at 10 Hz, two stages of 0.6 s
        est.update(0.1 * k, True, s, thrust, v0, np.zeros(3), M_D, M_L)
    assert np.allclose(est.d_L, d_L, atol=1e-3) and np.allclose(est.d_D, d_D, atol=1e-3)
    held = est.d_L.copy()
    s2, thrust2, _ = scene(np.zeros(3), np.zeros(3))
    for k in range(151, 201):                  # not gated: frozen
        est.update(0.1 * k, False, s2, thrust2, v0, np.zeros(3), M_D, M_L)
    assert np.allclose(est.d_L, held)
    big = DisturbanceEstimator(N, tau=0.1, bound_load=0.3)
    for k in range(50):
        big.update(0.1 * k, True, s, thrust, v0, np.zeros(3), M_D, M_L)
    assert big.railed and abs(big.d_L[0] + 0.3) < 1e-12
