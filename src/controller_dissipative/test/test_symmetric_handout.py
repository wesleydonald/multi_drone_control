"""Symmetric hand-out: the incumbents' balanced-tension solution must not STEP at a
weld (docs: DissipativeParams.handout_tension_blend; learning.txt 2026-09-09).

Setup mirrors the sim: three tethers on the 0.25 m rim at 0/120/240 deg, a newcomer
welded at the gap centre (180 deg), balanced tensions, soft hand-out. Each test names
the property that keeps the load level through the transition.
"""
import numpy as np
import pytest

from controller_dissipative.dissipative_network import (DissipativeNetwork,
                                                        DissipativeParams)
from controller_load_mpc.geometry import attach_points

G, CABLE, R_RIM, ATT_Z = 9.81, 0.5, 0.25, 0.025
LOAD_Q = np.array([1.0, 0.0, 0.0, 0.0])


def _fleet(blend, t_handout=10.0):
    params = DissipativeParams(balanced_tensions=True, T_handout=t_handout,
                               handout_tension_blend=blend)
    rho = attach_points(4, R_RIM, ATT_Z)
    rho[3] = np.array([R_RIM * np.cos(np.pi), R_RIM * np.sin(np.pi), ATT_Z])
    net = DissipativeNetwork(4, rho, CABLE, 0.6, 0.4, G, params)
    p_des = np.array([0.0, 0.0, 0.6])
    e = np.radians(45.0)
    seeds = []
    for i in range(4):
        az = rho[i][:2] / np.linalg.norm(rho[i][:2])
        seeds.append(p_des + rho[i] + CABLE * np.array([az[0] * np.cos(e),
                                                       az[1] * np.cos(e), np.sin(e)]))
    net.seed(seeds)
    net.detach(3)                               # newcomer not on the load yet
    for _ in range(50):
        net.step(p_des, LOAD_Q, np.zeros(3), p_des, 0.1)
    return net, p_des, rho


def _tensions(net, p_des):
    R = net._frame_rot(LOAD_Q)
    return net._solve_tensions(R, p_des)


def _weld(net, p_des, rho):
    hover = p_des + rho[3] + np.array([0.0, 0.0, CABLE])   # taut, straight above its rim point
    net.attach(3, hover, handout=True)


def test_incumbents_do_not_step_at_the_weld():
    """At handout 0 the incumbents carry exactly their pre-weld solution and the
    newcomer carries nothing -- the one-tick tension step is gone."""
    net, p_des, rho = _fleet(blend=True)
    before = _tensions(net, p_des)
    _weld(net, p_des, rho)
    after = _tensions(net, p_des)
    for i in range(3):
        assert abs(after[i] - before[i]) < 1e-9
    assert after[3] == pytest.approx(0.0, abs=1e-9)


def test_old_behaviour_steps_when_blend_is_off():
    """The A/B control: with the blend off, the weld steps the incumbents (the
    mechanism the Aug 9 weld ladder convicted)."""
    net, p_des, rho = _fleet(blend=False)
    before = _tensions(net, p_des)
    _weld(net, p_des, rho)
    after = _tensions(net, p_des)
    assert max(abs(after[i] - before[i]) for i in range(3)) > 0.1
    assert after[3] > 0.1


def test_every_intermediate_is_wrench_balanced():
    """Through the whole ramp the fleet still supports the weight with ~zero moment:
    both endpoints solve the same wrench, so their blend does too."""
    net, p_des, rho = _fleet(blend=True, t_handout=5.0)
    _weld(net, p_des, rho)
    for _ in range(60):                          # 6 s at 10 Hz > T_handout
        net.step(p_des, LOAD_Q, np.zeros(3), p_des, 0.1)
        F, M, rF, rM = net.net_wrench(LOAD_Q, p_des)
        assert abs(rF[2]) < 0.05 * 0.4 * G       # vertical support within 5 %
        assert np.linalg.norm(rM[:2]) < 0.06     # level (same bar as harness Test G)


def test_ramp_is_continuous_and_ends_at_the_full_solution():
    """Incumbent tension changes by a small amount per tick (no jumps) and the blend
    lands exactly on the unblended full-fleet solution when the hand-out completes."""
    net, p_des, rho = _fleet(blend=True, t_handout=5.0)
    _weld(net, p_des, rho)
    prev = _tensions(net, p_des)
    max_step = 0.0
    for _ in range(60):
        net.step(p_des, LOAD_Q, np.zeros(3), p_des, 0.1)
        cur = _tensions(net, p_des)
        max_step = max(max_step, max(abs(cur[i] - prev[i]) for i in range(4)))
        prev = cur
    assert max_step < 0.15                       # N per 0.1 s tick, vs a ~1 N step before
    assert net.handout[3] == pytest.approx(1.0)
    R = net._frame_rot(LOAD_Q)
    full = net._solve_subset([0, 1, 2, 3], R, p_des)
    for i in range(4):
        assert prev[i] == pytest.approx(full[i], abs=1e-9)
