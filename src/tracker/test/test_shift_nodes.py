"""_shift_nodes (card 2026-10-04_inner_loop_aware_planner): the planner horizon advanced by the time
since it arrived, by linear interpolation between nodes, the last node held."""
import numpy as np

from tracker.tracker_node import _shift_nodes


def test_shift_interpolates_and_holds_the_last_node():
    arr = np.array([[0.0, 1.0], [1.0, 1.0], [2.0, 1.0]])
    out = _shift_nodes(arr, 0.05, 0.1)
    assert np.allclose(out[:, 0], [0.5, 1.5, 2.0]) and np.allclose(out[:, 1], 1.0)


def test_no_shift_without_elapsed_time_or_array():
    arr = np.arange(6.0).reshape(3, 2)
    assert _shift_nodes(arr, 0.0, 0.1) is arr and _shift_nodes(None, 0.05, 0.1) is None
