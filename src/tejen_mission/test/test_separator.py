import math

import numpy as np
import pytest

from tejen_mission.cooperative_trajectory import SinusoidalLineTrajectory
from tejen_mission.convex_geometry import (
    box_vertices_from_center,
    rotate_vertices,
    rotation_z,
    swept_box_vertices_linear,
)
from tejen_mission.separator import build_separation_lp, glpk_available, solve_separator
from tejen_mission.time_indexed_hulls import AxisAlignedEnvelope, build_sinusoidal_hulls, uniform_intervals


def _assert_plane_valid(result, first, second, tolerance=1e-7):
    assert result.feasible
    assert result.plane is not None
    n = result.plane.normal
    d = result.plane.offset
    assert np.max(first @ n + d) <= -1.0 + tolerance
    assert np.min(second @ n + d) >= 1.0 - tolerance
    assert result.geometric_gap_m is not None
    assert result.geometric_gap_m > 0.0


def test_lp_builder_matches_canonical_two_sided_formulation():
    first = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    second = np.array([[2.0, 0.0, 0.0]])
    lp = build_separation_lp(first, second)
    np.testing.assert_allclose(lp.a_ub[0], [0.0, 0.0, 0.0, 1.0])
    np.testing.assert_allclose(lp.a_ub[1], [0.0, 1.0, 0.0, 1.0])
    np.testing.assert_allclose(lp.a_ub[2], [-2.0, 0.0, 0.0, -1.0])
    np.testing.assert_allclose(lp.b_ub, -np.ones(3))


def test_disjoint_boxes_are_separable():
    first = box_vertices_from_center(np.array([0.0, 0.0, 1.0]), np.array([0.2, 0.2, 0.2]))
    second = box_vertices_from_center(np.array([1.0, 0.1, 1.0]), np.array([0.2, 0.2, 0.2]))
    result = solve_separator(first, second, backend="scipy")
    _assert_plane_valid(result, first, second)


def test_near_touching_boxes_are_still_separable():
    first = box_vertices_from_center(np.array([0.0, 0.0, 1.0]), np.array([0.25, 0.20, 0.15]))
    second = box_vertices_from_center(np.array([0.5005, 0.0, 1.0]), np.array([0.25, 0.20, 0.15]))
    result = solve_separator(first, second, backend="scipy")
    _assert_plane_valid(result, first, second)


def test_touching_boxes_are_infeasible():
    first = box_vertices_from_center(np.array([0.0, 0.0, 1.0]), np.array([0.25, 0.20, 0.15]))
    second = box_vertices_from_center(np.array([0.50, 0.0, 1.0]), np.array([0.25, 0.20, 0.15]))
    result = solve_separator(first, second, backend="scipy")
    assert not result.feasible
    assert result.plane is None


def test_overlapping_rotated_boxes_are_infeasible():
    first = box_vertices_from_center(np.array([0.0, 0.0, 1.0]), np.array([0.30, 0.22, 0.18]))
    second = rotate_vertices(
        box_vertices_from_center(np.array([0.28, 0.06, 1.0]), np.array([0.28, 0.18, 0.16])),
        rotation_z(math.radians(31.0)),
    )
    result = solve_separator(first, second, backend="scipy")
    assert not result.feasible


def test_rotated_disjoint_polyhedra_are_separable():
    first = rotate_vertices(
        box_vertices_from_center(np.array([0.0, 0.0, 1.0]), np.array([0.24, 0.16, 0.12])),
        rotation_z(math.radians(22.0)),
    )
    second = rotate_vertices(
        box_vertices_from_center(np.array([0.75, 0.55, 1.12]), np.array([0.22, 0.14, 0.16])),
        rotation_z(math.radians(-37.0)),
    )
    result = solve_separator(first, second, backend="scipy")
    _assert_plane_valid(result, first, second)


def test_swapping_sets_preserves_separability():
    first = box_vertices_from_center(np.array([0.0, 0.0, 1.0]), np.array([0.2, 0.2, 0.2]))
    second = box_vertices_from_center(np.array([0.9, 0.3, 1.1]), np.array([0.15, 0.18, 0.15]))
    forward = solve_separator(first, second, backend="scipy")
    reverse = solve_separator(second, first, backend="scipy")
    _assert_plane_valid(forward, first, second)
    _assert_plane_valid(reverse, second, first)


def test_r2_sinusoidal_time_hulls_can_be_separated_from_far_query():
    obstacle = SinusoidalLineTrajectory()
    envelope = AxisAlignedEnvelope(np.array([0.105, 0.105, 0.060]), 0.0)
    edges = uniform_intervals(0.0, 7.291666666666667, 4)
    hulls = build_sinusoidal_hulls(obstacle, envelope, edges)
    query = box_vertices_from_center(np.array([-0.5, -0.6, 1.1]), np.array([0.12, 0.12, 0.10]))
    for hull in hulls:
        result = solve_separator(query, hull.vertices, backend="scipy")
        _assert_plane_valid(result, query, hull.vertices)


def test_r2_sinusoidal_hull_detects_intentional_overlap():
    obstacle = SinusoidalLineTrajectory()
    envelope = AxisAlignedEnvelope(np.array([0.105, 0.105, 0.060]), 0.0)
    edges = uniform_intervals(0.0, 7.291666666666667, 4)
    hulls = build_sinusoidal_hulls(obstacle, envelope, edges)
    selected = hulls[1]
    query = box_vertices_from_center(0.5 * (selected.lower + selected.upper), np.array([0.04, 0.04, 0.03]))
    result = solve_separator(query, selected.vertices, backend="scipy")
    assert not result.feasible


def test_constant_velocity_swept_hull_separation_and_overlap():
    half = np.array([0.12, 0.10, 0.075])
    swept = swept_box_vertices_linear(
        np.array([0.5, 1.7, 1.3]),
        np.array([0.9, 1.5, 1.4]),
        half,
    )
    far_query = box_vertices_from_center(np.array([-0.2, 0.0, 1.0]), np.array([0.15, 0.15, 0.12]))
    _assert_plane_valid(solve_separator(far_query, swept, backend="scipy"), far_query, swept)

    overlap_query = box_vertices_from_center(np.array([0.7, 1.6, 1.35]), np.array([0.05, 0.05, 0.05]))
    assert not solve_separator(overlap_query, swept, backend="scipy").feasible


def test_invalid_backend_is_rejected():
    a = box_vertices_from_center(np.zeros(3), np.ones(3) * 0.1)
    b = box_vertices_from_center(np.ones(3), np.ones(3) * 0.1)
    with pytest.raises(ValueError):
        solve_separator(a, b, backend="not-a-backend")


@pytest.mark.skipif(not glpk_available(), reason="native GLPK shared library not installed")
def test_native_glpk_backend_when_available():
    first = box_vertices_from_center(np.array([0.0, 0.0, 1.0]), np.array([0.2, 0.2, 0.2]))
    second = box_vertices_from_center(np.array([1.0, 0.1, 1.0]), np.array([0.2, 0.2, 0.2]))
    result = solve_separator(first, second, backend="glpk")
    _assert_plane_valid(result, first, second)
    assert result.backend == "glpk"
