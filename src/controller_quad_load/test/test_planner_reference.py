"""Tests for set_planner_reference — in particular the terminal cost (experiment T1).

Background: `yref_N[3:6]` was never assigned, so while every stage cost tracked
`ref_vel[j]`, the TERMINAL cost asked for velocity zero — telling the drone to stop at
the end of every 2 s horizon while flying a circle at 0.6 m/s. That is a standing
deceleration bias and a leading suspect for the 0.589 s tracker-stage lag in
DISSIPATIVE_TRACKING_ISSUE.md.

These tests pin down both behaviours so the A/B is unambiguous, and so the historical
default cannot change by accident.

    python3 -m pytest src/controller_quad_load/test/test_planner_reference.py -v
"""
import numpy as np
import pytest

from controller_quad_load.acados import set_planner_reference

N = 20
EST = [30.0, 0.0, 0.12, 70.0, 670.0, 0.5]


class StubSolver:
    """Records what would have been handed to acados. `set_planner_reference` only
    calls .set(stage, key, value), so this is the whole interface."""

    def __init__(self):
        self.yref = {}
        self.p = {}

    def set(self, stage, key, value):
        if key == 'yref':
            self.yref[stage] = np.asarray(value, float).copy()
        elif key == 'p':
            self.p[stage] = np.asarray(value, float).copy()


def moving_reference(speed=0.6):
    """A reference travelling along +x at a constant speed — the case where the
    terminal velocity reference actually matters."""
    dt = 0.1
    pos = np.zeros((N + 1, 3))
    pos[:, 0] = speed * dt * np.arange(N + 1)
    pos[:, 2] = 1.0
    vel = np.zeros((N + 1, 3))
    vel[:, 0] = speed
    acc = np.tile(np.array([0.0, 0.0, 9.81]), (N + 1, 1))
    return pos, vel, acc


def run(terminal_vel_ref, speed=0.6):
    s = StubSolver()
    pos, vel, acc = moving_reference(speed)
    set_planner_reference(s, pos, vel, acc, N, EST,
                          terminal_vel_ref=terminal_vel_ref)
    return s, vel


# ── the historical behaviour, pinned ─────────────────────────────────────────

def test_default_leaves_terminal_velocity_at_zero():
    """The default must reproduce the pre-2026-08-04 behaviour exactly, so that
    enabling T1 is the only difference between two compared runs."""
    s, _ = run(terminal_vel_ref=False)
    assert np.allclose(s.yref[N][3:6], 0.0)


def test_stage_costs_always_track_the_reference_velocity():
    """Stage costs were never the problem — verify they still are not."""
    for flag in (False, True):
        s, vel = run(terminal_vel_ref=flag)
        for j in range(N):
            assert np.allclose(s.yref[j][3:6], vel[j]), f"stage {j}, flag={flag}"


# ── T1 ───────────────────────────────────────────────────────────────────────

def test_enabling_t1_sets_the_terminal_velocity_reference():
    s, vel = run(terminal_vel_ref=True)
    assert np.allclose(s.yref[N][3:6], vel[N])
    assert s.yref[N][3] == pytest.approx(0.6)


def test_t1_is_the_only_difference():
    """Nothing else about the reference may change, or an A/B measures two things."""
    a, _ = run(terminal_vel_ref=False)
    b, _ = run(terminal_vel_ref=True)
    for stage in range(N + 1):
        assert np.allclose(a.p[stage], b.p[stage]), f"parameters differ at stage {stage}"
    for stage in range(N):
        assert np.allclose(a.yref[stage], b.yref[stage]), f"stage cost differs at {stage}"
    # Only the terminal velocity slots may differ. Which of them actually do
    # depends on the reference: travelling along +x, the y and z components are
    # zero in BOTH modes, so index 3 alone changes.
    diff = set(np.flatnonzero(~np.isclose(a.yref[N], b.yref[N])).tolist())
    assert diff, "enabling T1 changed nothing — the flag is not wired through"
    assert diff <= {3, 4, 5}, (
        f"expected only the terminal velocity slots (3-5) to differ, got {sorted(diff)}")


def test_hover_is_unaffected_by_the_flag():
    """With a stationary reference the two modes are identical, so hover, LAND and
    every detach/attach test are bit-identical regardless of the flag."""
    a, _ = run(terminal_vel_ref=False, speed=0.0)
    b, _ = run(terminal_vel_ref=True, speed=0.0)
    for stage in range(N + 1):
        assert np.allclose(a.yref[stage], b.yref[stage])


# ── general contract ─────────────────────────────────────────────────────────

def test_terminal_position_and_throttle_are_always_set():
    s, _ = run(terminal_vel_ref=False)
    pos, _, _ = moving_reference()
    assert np.allclose(s.yref[N][0:3], pos[N])
    assert s.yref[N][11] > 0.0, "terminal throttle feedforward must be set"


def test_every_stage_gets_a_reference():
    s, _ = run(terminal_vel_ref=True)
    assert set(s.yref) == set(range(N + 1))
    assert set(s.p) == set(range(N + 1))
