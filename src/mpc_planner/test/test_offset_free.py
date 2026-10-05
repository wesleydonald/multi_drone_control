"""OffsetFreeObserver (card 2026-10-04_offset_free_innovation): on a point-mass load under an
unknown constant force, a planner that plans with the estimate drives the innovation, and the
estimate converges to the force with its time constant; frozen when not gated or without a plan;
bounded."""
import numpy as np

from mpc_planner.offset_free import OffsetFreeObserver, predicted_velocity

M, DT = 0.86, 0.1


def test_predicted_velocity_interpolates_the_horizon():
    V = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.3, 0.0, 0.0]])
    assert np.allclose(predicted_velocity(1.0, DT, V, 1.05), [0.05, 0.0, 0.0])
    assert np.allclose(predicted_velocity(1.0, DT, V, 1.15), [0.2, 0.0, 0.0])
    assert predicted_velocity(1.0, DT, V, 0.9) is None and predicted_velocity(1.0, DT, V, 1.3) is None


def fly(d_true, ticks, gated=True, tau=1.0):
    """A load at rest that the plan keeps at rest by countering d_hat (planned accel 0 with the
    estimate in the model); the true force accelerates it by (d_true - d_hat)/m for one tick,
    then the plan re-pins (the real loop holds it, the residual shows in the innovation)."""
    obs = OffsetFreeObserver(M, tau=tau)
    for k in range(ticks):
        t = k * DT
        obs.set_plan(t, DT, np.zeros((21, 3)))
        v = (np.asarray(d_true) - obs.d) / M * DT
        obs.update(t + DT, gated, v)
    return obs


def test_converges_to_the_unexplained_force_with_its_time_constant():
    d_true = np.array([-1.0, 0.0, 0.4])
    assert np.allclose(fly(d_true, 100).d, d_true, atol=1e-3)
    one_tau = fly(d_true, 11)          # the first gated tick is skipped, then 1.0 s = tau at k = 0.1
    assert np.allclose(one_tau.d[[0, 2]] / d_true[[0, 2]], 1.0 - 0.9 ** 10, atol=1e-9)


def test_the_first_tick_after_a_freeze_is_not_integrated():
    obs = OffsetFreeObserver(M, tau=1.0)
    obs.set_plan(0.0, DT, np.zeros((21, 3)))
    obs.update(DT, True, [0.1, 0.0, 0.0])
    assert np.allclose(obs.d, 0.0) and obs.raw[0] > 0.0
    obs.set_plan(DT, DT, np.zeros((21, 3)))
    obs.update(2 * DT, True, [0.1, 0.0, 0.0])
    assert obs.d[0] > 0.0


def test_frozen_when_not_gated_or_without_a_plan():
    obs = fly([0.5, 0.0, 0.0], 50, gated=False)
    assert np.allclose(obs.d, 0.0) and obs.raw is not None
    obs = OffsetFreeObserver(M)
    obs.update(0.1, True, [1.0, 0.0, 0.0])            # no plan published
    assert np.allclose(obs.d, 0.0) and obs.raw is None
    obs.set_plan(0.0, DT, np.zeros((21, 3)))
    obs.update(0.5, True, [1.0, 0.0, 0.0])            # gap longer than max_gap
    assert np.allclose(obs.d, 0.0)


def test_bounded():
    obs = fly([10.0, 0.0, 0.0], 100)
    assert obs.railed and abs(obs.d[0] - 3.0) < 1e-12


def test_window_converges_on_a_load_the_plans_do_not_hold():
    """Window 0.5 s: a load accelerating freely under d_true, each tick re-planned from the
    measured state with the current estimate; the estimate converges to d_true."""
    d_true = np.array([0.6, 0.0, -0.3])
    obs = OffsetFreeObserver(M, tau=1.0, window=0.5)
    nodes = np.arange(21)[:, None] * DT
    for k in range(120):
        t = k * DT
        p, v = 0.5 * d_true / M * t ** 2, d_true / M * t
        obs.update(t, True, v, p)
        P = p + v * nodes + 0.5 * obs.d / M * nodes ** 2
        obs.set_plan(t, DT, v + obs.d / M * nodes, P)
    assert np.allclose(obs.d, d_true, atol=1e-3)
