"""Pure payload-MPC runtime cost-mode helpers for M2.

The canonical FREE_SWING matrices are constructed here once and reused by the OCP
builder and runtime mode switcher.  This prevents a second source of truth for MPC
weights while allowing only the payload-swing block to be suppressed in modes where
the free-pendulum cost is not physically meaningful.
"""

from __future__ import annotations

from enum import Enum

import numpy as np


class PayloadMpcMode(str, Enum):
    VERTICAL_TAKEOFF = "VERTICAL_TAKEOFF"
    FREE_SWING = "FREE_SWING"
    ATTACH_APPROACH = "ATTACH_APPROACH"
    ATTACH_PROOF = "ATTACH_PROOF"
    ATTACHED_HOLD = "ATTACHED_HOLD"
    DETACHED_RETREAT = "DETACHED_RETREAT"
    LANDING = "LANDING"


_ZERO_SWING_MODES = {
    PayloadMpcMode.VERTICAL_TAKEOFF,
    PayloadMpcMode.ATTACH_PROOF,
    PayloadMpcMode.ATTACHED_HOLD,
}


def parse_payload_mpc_mode(value) -> PayloadMpcMode:
    if isinstance(value, PayloadMpcMode):
        return value
    text = str(value).strip().upper()
    try:
        return PayloadMpcMode(text)
    except ValueError as exc:
        valid = ", ".join(mode.value for mode in PayloadMpcMode)
        raise ValueError(f"Unknown payload MPC mode {value!r}; expected one of: {valid}") from exc


def base_payload_cost_matrices(
    xy_integral_weight: float = 15.0,
    xy_integral_terminal_weight: float = 30.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Return the exact canonical payload-OCP running and terminal weights."""
    W = np.diag([
        130.0, 130.0, 40.0,
        2.0, 2.0, 2.0,
        0.2, 0.2, 0.2,
        2e-4, 2e-4, 2e-4, 2e-4,
        0.1, 0.1, 5.0, 0.1,
        0.5, 0.5, 5.0,
        20.0, 20.0,
        2.0, 2.0,
        float(xy_integral_weight), float(xy_integral_weight),
    ])
    W_e = np.diag([
        200.0, 200.0, 60.0,
        2.0, 2.0, 2.0,
        0.2, 0.2, 0.2,
        2e-4, 2e-4, 2e-4, 2e-4,
        0.5, 0.5, 5.0,
        20.0, 20.0,
        2.0, 2.0,
        float(xy_integral_terminal_weight), float(xy_integral_terminal_weight),
    ])
    return W, W_e


def payload_cost_matrices_for_mode(
    mode,
    base_running_cost: np.ndarray,
    base_terminal_cost: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    parsed = parse_payload_mpc_mode(mode)
    W = np.asarray(base_running_cost, dtype=float).copy()
    W_e = np.asarray(base_terminal_cost, dtype=float).copy()
    if W.shape != (26, 26):
        raise ValueError(f"base_running_cost must have shape (26, 26), got {W.shape}")
    if W_e.shape != (22, 22):
        raise ValueError(f"base_terminal_cost must have shape (22, 22), got {W_e.shape}")
    if parsed in _ZERO_SWING_MODES:
        # NONLINEAR_LS output layout in acados.py:
        # running payload state [phi, theta, phi_dot, theta_dot] -> 20:24
        # terminal payload state -> 16:20
        W[20:24, 20:24] = 0.0
        W_e[16:20, 16:20] = 0.0
    return W, W_e


def apply_payload_mpc_mode(
    solver,
    *,
    horizon_stages: int,
    mode,
    base_running_cost: np.ndarray,
    base_terminal_cost: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    parsed = parse_payload_mpc_mode(mode)
    W, W_e = payload_cost_matrices_for_mode(parsed, base_running_cost, base_terminal_cost)
    N = int(horizon_stages)
    if N < 1:
        raise ValueError("horizon_stages must be at least one")
    for stage in range(N):
        solver.cost_set(stage, "W", W)
    solver.cost_set(N, "W", W_e)
    return W, W_e
