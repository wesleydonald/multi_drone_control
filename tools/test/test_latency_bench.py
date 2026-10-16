"""Unit tests for tools/latency_bench.py's plant, tracker reference and plan-age measure (no solver)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import latency_bench as B  # noqa: E402

RHO = [np.array([0.225 * math.cos(a), 0.225 * math.sin(a), 0.02]) for a in (0.0, 2.0944, 4.18879)]


def _hung(plant, elev=60.0):
    """Pivots that hold the ring level at (0, 0, 1) with rods at elev above horizontal."""
    plant.x = np.array([0.0, 0.0, 1.0])
    ph = math.radians(elev)
    return np.array([plant.x + r + 0.55 * np.array([math.cos(math.atan2(r[1], r[0])) * math.cos(ph),
                                                    math.sin(math.atan2(r[1], r[0])) * math.cos(ph), math.sin(ph)])
                     for r in RHO])


@pytest.mark.parametrize('kind', ['p0', 'p1'])
def test_ring_hangs_still_from_fixed_pivots(kind):
    """The ring released level under three fixed pivots settles within the rods' stretch and stays level."""
    plant = B.RingPlant(RHO, 0.55, kind, planner_inertia=(0.0235, 0.0235, 0.047))
    piv = _hung(plant)
    for _ in range(4000):
        plant.step(B.DT_SIM, piv, np.zeros((3, 3)))
    assert abs(plant.x[2] - 1.0) < 0.003 and np.linalg.norm(plant.x[:2]) < 1e-6
    assert plant.tilt_deg() < 0.1 and np.linalg.norm(plant.v) < 1e-3


def test_p1_carries_the_rods_mass_at_the_ring_end():
    p0 = B.RingPlant(RHO, 0.55, 'p0', planner_inertia=(0.0235, 0.0235, 0.047))
    p1 = B.RingPlant(RHO, 0.55, 'p1')
    me = 0.075 * (1 - 0.147 / 0.55)
    assert p0.m == pytest.approx(0.86) and p1.m == pytest.approx(0.86 + 3 * me)
    assert p1.I[2, 2] == pytest.approx(4.431e-2 + 3 * me * 0.225 ** 2, rel=1e-6)
    assert p1.joint_c == 0.1 and p0.joint_c == 0.0


def test_ring_follows_pivots_moving_together():
    """Pivots translating at 0.3 m/s: the ring moves with them (a rigid formation; it sways after the ramp, so the
    last second is averaged)."""
    plant = B.RingPlant(RHO, 0.55, 'p1')
    piv0 = _hung(plant)
    v = np.array([0.3, 0.0, 0.0])
    vx, x_lag = [], []
    for k in range(8000):
        t = k * B.DT_SIM
        ramp = min(t / 1.0, 1.0)
        off = v * (0.5 * t * t if t < 1.0 else 0.5 + (t - 1.0))
        plant.step(B.DT_SIM, piv0 + off, np.tile(v * ramp, (3, 1)))
        if k >= 6000:
            vx.append(plant.v[0])
            x_lag.append(plant.x[0] - off[0])
    assert np.mean(vx) == pytest.approx(0.3, abs=0.01)
    assert abs(np.mean(x_lag)) < 0.02


def test_flown_reference_is_node_0_at_zero_age_and_interpolates_between_nodes():
    P = np.column_stack([np.arange(21) * 0.03, np.zeros(21), np.ones(21)])
    V = np.tile([0.3, 0.0, 0.0], (21, 1))
    A = np.zeros((21, 3))
    d = B.densify((P, V, A), 0.1)
    assert B.flown(d, 0.0)[0] == pytest.approx(P[0])
    assert B.flown(d, 0.15)[0][0] == pytest.approx(0.045, abs=1e-9)
    assert B.flown(d, 9.0)[0] == pytest.approx(P[-1])


def test_plan_age_D_recovers_a_known_delay():
    """lag_metrics' convention: D is how far the pose LEADS its node-0 reference (node 0 is pinned to the state
    the plan was measured from, so the reference is that old)."""
    t = np.arange(0.0, 10.0, 0.02)
    piv = np.column_stack([0.5 * np.sin(0.6 * t), 0.5 * (1 - np.cos(0.6 * t)), np.ones(t.size)])
    ref = np.column_stack([0.5 * np.sin(0.6 * (t - 0.08)), 0.5 * (1 - np.cos(0.6 * (t - 0.08))), np.ones(t.size)])
    assert B.plan_age_D({'t': list(t), 'piv': list(piv), 'ref': list(ref)}) == pytest.approx(0.08, abs=0.006)
