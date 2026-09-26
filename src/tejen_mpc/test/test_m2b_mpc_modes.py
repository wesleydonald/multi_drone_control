"""Tests-first contract for M2B runtime payload-MPC cost modes."""

from __future__ import annotations

import numpy as np
import pytest

from tejen_mpc.mpc_modes import (
    PayloadMpcMode,
    apply_payload_mpc_mode,
    base_payload_cost_matrices,
    payload_cost_matrices_for_mode,
)


def _base():
    return base_payload_cost_matrices(15.0, 30.0)


def test_free_swing_is_exactly_the_canonical_existing_cost():
    W0, We0 = _base()
    W, We = payload_cost_matrices_for_mode(PayloadMpcMode.FREE_SWING, W0, We0)
    assert np.array_equal(W, W0)
    assert np.array_equal(We, We0)


@pytest.mark.parametrize(
    "mode",
    [
        PayloadMpcMode.VERTICAL_TAKEOFF,
        PayloadMpcMode.ATTACH_PROOF,
        PayloadMpcMode.ATTACHED_HOLD,
    ],
)
def test_zero_swing_modes_change_only_running_and_terminal_payload_blocks(mode):
    W0, We0 = _base()
    W, We = payload_cost_matrices_for_mode(mode, W0, We0)

    expected_W = W0.copy()
    expected_W[20:24, 20:24] = 0.0
    expected_We = We0.copy()
    expected_We[16:20, 16:20] = 0.0

    assert np.array_equal(W, expected_W)
    assert np.array_equal(We, expected_We)
    # Position, attitude and integral weights are specifically protected.
    assert np.array_equal(W[:20, :20], W0[:20, :20])
    assert np.array_equal(W[24:26, 24:26], W0[24:26, 24:26])
    assert np.array_equal(We[:16, :16], We0[:16, :16])
    assert np.array_equal(We[20:22, 20:22], We0[20:22, 20:22])


@pytest.mark.parametrize(
    "mode",
    [
        PayloadMpcMode.ATTACH_APPROACH,
        PayloadMpcMode.DETACHED_RETREAT,
        PayloadMpcMode.LANDING,
    ],
)
def test_free_swing_family_preserves_existing_cost_exactly(mode):
    W0, We0 = _base()
    W, We = payload_cost_matrices_for_mode(mode, W0, We0)
    assert np.array_equal(W, W0)
    assert np.array_equal(We, We0)


def test_unknown_mode_is_rejected_instead_of_silent_fallback():
    W0, We0 = _base()
    with pytest.raises(ValueError):
        payload_cost_matrices_for_mode("banana", W0, We0)


class FakeSolver:
    def __init__(self):
        self.calls = []

    def cost_set(self, stage, field, value):
        self.calls.append((stage, field, np.asarray(value).copy()))


def test_apply_updates_every_running_stage_and_terminal_stage():
    W0, We0 = _base()
    solver = FakeSolver()
    W, We = apply_payload_mpc_mode(
        solver,
        horizon_stages=3,
        mode=PayloadMpcMode.ATTACHED_HOLD,
        base_running_cost=W0,
        base_terminal_cost=We0,
    )
    assert [(s, f) for s, f, _ in solver.calls] == [
        (0, "W"), (1, "W"), (2, "W"), (3, "W"),
    ]
    for _, _, value in solver.calls[:3]:
        assert np.array_equal(value, W)
    assert np.array_equal(solver.calls[-1][2], We)
