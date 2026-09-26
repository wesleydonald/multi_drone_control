"""Small, permanent convex-geometry helpers for separator visualization."""

from __future__ import annotations

from itertools import product

import numpy as np
from scipy.spatial import ConvexHull

from .separator import SeparatingPlane


def box_vertices_from_center(center: np.ndarray, half_extents: np.ndarray) -> np.ndarray:
    center = np.asarray(center, dtype=float).reshape(3)
    half = np.asarray(half_extents, dtype=float).reshape(3)
    if not np.all(np.isfinite(center)) or not np.all(np.isfinite(half)):
        raise ValueError("center and half_extents must be finite")
    if np.any(half <= 0.0):
        raise ValueError("half_extents must be positive")
    return np.asarray(
        [center + half * np.array(signs, dtype=float) for signs in product((-1.0, 1.0), repeat=3)],
        dtype=float,
    )


def rotate_vertices(vertices: np.ndarray, rotation: np.ndarray, origin: np.ndarray | None = None) -> np.ndarray:
    vertices = np.asarray(vertices, dtype=float)
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    if vertices.ndim != 2 or vertices.shape[1] != 3:
        raise ValueError("vertices must have shape (N, 3)")
    if origin is None:
        origin = np.mean(vertices, axis=0)
    origin = np.asarray(origin, dtype=float).reshape(3)
    return (vertices - origin) @ rotation.T + origin


def rotation_z(angle_rad: float) -> np.ndarray:
    c = float(np.cos(angle_rad))
    s = float(np.sin(angle_rad))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def triangular_faces(vertices: np.ndarray) -> np.ndarray:
    vertices = np.asarray(vertices, dtype=float)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or vertices.shape[0] < 4:
        raise ValueError("vertices must have shape (N,3), N>=4")
    hull = ConvexHull(vertices)
    return hull.simplices.copy()


def unique_hull_edges(vertices: np.ndarray) -> tuple[tuple[int, int], ...]:
    faces = triangular_faces(vertices)
    edges: set[tuple[int, int]] = set()
    for face in faces:
        for a, b in ((face[0], face[1]), (face[1], face[2]), (face[2], face[0])):
            edges.add(tuple(sorted((int(a), int(b)))))
    return tuple(sorted(edges))


def plane_patch_vertices(
    plane: SeparatingPlane,
    reference_points: np.ndarray,
    *,
    scale: float = 1.15,
    level: float = 0.0,
) -> np.ndarray:
    """Return four vertices of a square patch on ``n.T x + d = level``."""

    points = np.asarray(reference_points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
        raise ValueError("reference_points must have shape (N,3)")
    if scale <= 0.0:
        raise ValueError("scale must be positive")

    n = plane.unit_normal
    # Shift the closest point on the zero plane along n to reach the requested level.
    raw_norm = float(np.linalg.norm(plane.normal))
    anchor = plane.point_on_plane() + (float(level) / raw_norm) * n

    axis = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(axis, n))) > 0.85:
        axis = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, axis)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    v /= np.linalg.norm(v)

    centre = np.mean(points, axis=0)
    # Translate within the plane so the patch is centred near the geometry.
    to_centre = centre - anchor
    anchor = anchor + u * np.dot(to_centre, u) + v * np.dot(to_centre, v)

    projected_u = points @ u
    projected_v = points @ v
    half = max(float(np.ptp(projected_u)), float(np.ptp(projected_v)), 0.4) * 0.5 * scale
    return np.asarray(
        [anchor - half * u - half * v, anchor + half * u - half * v, anchor + half * u + half * v, anchor - half * u + half * v],
        dtype=float,
    )


def swept_box_vertices_linear(
    start_center: np.ndarray,
    end_center: np.ndarray,
    half_extents: np.ndarray,
) -> np.ndarray:
    """Vertices whose convex hull exactly encloses a linearly translated AABB."""

    return np.vstack(
        (
            box_vertices_from_center(start_center, half_extents),
            box_vertices_from_center(end_center, half_extents),
        )
    )
