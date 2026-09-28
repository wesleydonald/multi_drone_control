"""M2 join stall (T0023, T0025, T0032, T0033): the cached C++ window is cut mid-transit, so
advance_reference_window pads past its end with a moving terminal state (position frozen,
velocity ~0.22 m/s). The recovery check then compares every fresh C++ reference, which
starts from the held drone at <= ~0.09 m/s, against that phantom velocity and rejects it
forever (online_join_planner.py C1F.2b recovery, default tolerances 0.03 m / 0.10 m/s /
0.5 m/s^2). Fixed: the padding is a stopped hold."""
import unittest

import numpy as np

from tejen_mission.c1f2_handoff import advance_reference_window, handoff_allowed, stage0_errors
from tejen_mission.reference_generators import TrajectoryReference

DT = 1.0 / 30.0
V_TRANSIT = np.array([0.028, -0.206, 0.082])       # T0032 drone_0 ref_v at the freeze (0.223 m/s)


def moving_window(n=61):
    """A 2 s slice from the middle of a 5.2 s transit: still moving at its last sample."""
    t = np.arange(n, dtype=float) * DT
    p0 = np.array([-0.177, 0.914, 0.603])
    return TrajectoryReference(p0 + np.outer(t, V_TRANSIT),
                               np.repeat(V_TRANSIT.reshape(1, 3), n, axis=0),
                               np.zeros((n, 3)))


def candidate_from(p, v, n=61):
    return TrajectoryReference(np.repeat(np.asarray(p, float).reshape(1, 3), n, axis=0),
                               np.repeat(np.asarray(v, float).reshape(1, 3), n, axis=0),
                               np.zeros((n, 3)))


class PaddingDeadlockTests(unittest.TestCase):
    def cached_after_the_window(self):
        return advance_reference_window(moving_window(), elapsed_s=7.0, sample_dt_s=DT,
                                        output_count=61)

    def test_padding_keeps_the_frozen_position(self):
        cached = self.cached_after_the_window()
        np.testing.assert_allclose(cached.positions[0], moving_window().positions[-1])

    def test_padding_past_the_window_is_a_stopped_hold(self):
        np.testing.assert_allclose(self.cached_after_the_window().velocities[0], 0.0, atol=1e-9)

    def test_a_fresh_reference_from_the_held_drone_is_accepted(self):
        cached = self.cached_after_the_window()
        # the backend's replan starts where the drone holds, moving at most ~0.09 m/s
        fresh = candidate_from(cached.positions[0], 0.09 * V_TRANSIT / np.linalg.norm(V_TRANSIT))
        self.assertTrue(handoff_allowed(stage0_errors(cached, fresh), position_tolerance_m=0.03,
                                        velocity_tolerance_mps=0.10,
                                        acceleration_tolerance_mps2=0.5))


if __name__ == '__main__':
    unittest.main()
