"""The next-tick command (card docs/experiments/2026-10-14_tracker_timing.md)."""
import numpy as np

from tracker.tracker_node import DT, next_tick_command


def test_next_tick_command_integrates_the_first_stage_rate():
    u0 = np.array([0.1, -0.2, 0.45, 0.0])
    rate = np.array([1.0, -1.0, 0.5, 0.2])
    assert np.allclose(next_tick_command(u0, rate, DT), u0 + rate * DT)


def test_a_bounded_rate_moves_the_throttle_by_at_most_its_bound_per_tick():
    """The model allows |throttle_dot| <= 0.5/s: one 20 ms tick may then move it 0.01, not node 1's 0.05."""
    u0 = np.array([0.0, 0.0, 0.40, 0.0])
    for r in (-0.5, 0.5):
        u = next_tick_command(u0, [0.0, 0.0, r, 0.0], DT)
        assert abs(u[2] - u0[2]) <= 0.5 * DT + 1e-12


def test_only_the_throttle_changes_and_only_once_airborne():
    """Critic must-fixes: roll, pitch and yaw keep node 1's command; the spool (before the stand is left) too."""
    import inspect
    from tracker import tracker_node
    src = inspect.getsource(tracker_node.Controller.control_loop)
    assert 'u[2] = next_tick_command(' in src and 'if self._next_tick_live():' in src
    live = inspect.getsource(tracker_node.Controller._next_tick_live)
    assert 'self.takeoff_requested' in live and 'self._kt_airborne()' in live
