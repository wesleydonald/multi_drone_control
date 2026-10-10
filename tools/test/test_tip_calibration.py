"""tools/tip_calibration.py: the tip in the ring frame and the two launch values."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from tip_calibration import SETPOINT_TODAY, in_ring_frame, summarise  # noqa: E402


def _yaw(deg):
    h = np.radians(deg) / 2.0
    return (np.cos(h), 0.0, 0.0, np.sin(h))


def test_ring_frame_undoes_the_ring_pose():
    ring, q = (1.0, 2.0, 0.5), _yaw(90.0)
    # plate 9 is ring +y; with the ring yawed 90 deg it lies along world -x
    rel = in_ring_frame((1.0 - 0.225, 2.0, 0.5 + 0.03), ring, q)
    assert np.allclose(rel, [0.0, 0.225, 0.03], atol=1e-9)


def test_launch_values_from_a_still_record():
    rng = np.random.default_rng(1)
    ring, q = (0.0, 0.0, 0.05), _yaw(0.0)
    s = [(0.01 * i, (0.0, 0.225, 0.05 + 0.025) + rng.normal(0, 2e-4, 3), ring, q) for i in range(100)]
    out = summarise(s, standoff=0.01)
    assert abs(out['h'] - 0.025) < 1e-3
    assert abs(out['weld_point_z'] - 0.035) < 1e-3
    assert abs(out['attach_tip_dz'] - (0.035 - SETPOINT_TODAY)) < 1e-3
    assert out['repeats'] == 0 and abs(out['rate_hz'] - 100.0) < 1.0


def test_repeats_and_gaps_are_reported():
    ring, q = (0.0, 0.0, 0.0), _yaw(0.0)
    s = [(0.01 * i, (0.0, 0.225, 0.03), ring, q) for i in range(10)] + [(0.5, (0.0, 0.225, 0.031), ring, q)]
    out = summarise(s, standoff=0.0)
    assert out['repeats'] == 9 and abs(out['longest_gap_s'] - 0.41) < 1e-9
