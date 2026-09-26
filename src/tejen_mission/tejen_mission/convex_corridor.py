"""ROS-independent convex safe-flight-corridor geometry."""


from __future__ import annotations
from itertools import combinations

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np

from .voxel_astar import SphereObstacle


def _vec3(value: Sequence[float], name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain exactly three finite values")
    return array.copy()


@dataclass(frozen=True)
class ConvexPolyhedron:
    """Convex region represented by A @ x <= b."""

    A: np.ndarray
    b: np.ndarray
    segment_start: np.ndarray
    segment_end: np.ndarray

    def __post_init__(self) -> None:
        A = np.asarray(self.A, dtype=float)
        b = np.asarray(self.b, dtype=float).reshape(-1)

        if A.ndim != 2 or A.shape[1] != 3:
            raise ValueError("A must have shape (M, 3)")
        if A.shape[0] == 0 or b.shape != (A.shape[0],):
            raise ValueError("b must have shape (M,) with M >= 1")
        if not np.all(np.isfinite(A)) or not np.all(np.isfinite(b)):
            raise ValueError("polyhedron half-spaces must be finite")

        normal_lengths = np.linalg.norm(A, axis=1)
        if np.any(normal_lengths <= 1e-12):
            raise ValueError("polyhedron contains a zero-length plane normal")

        # Store every plane with a unit normal. This makes geometric margins
        # and later mission/swarm half-space constraints straightforward.
        A = A / normal_lengths[:, None]
        b = b / normal_lengths

        object.__setattr__(self, "A", A)
        object.__setattr__(self, "b", b)
        object.__setattr__(
            self,
            "segment_start",
            _vec3(self.segment_start, "segment_start"),
        )
        object.__setattr__(
            self,
            "segment_end",
            _vec3(self.segment_end, "segment_end"),
        )

    def contains(self, point: Sequence[float], tolerance: float = 1e-9) -> bool:
        point = _vec3(point, "point")
        return bool(np.all(self.A @ point <= self.b + float(tolerance)))

    def minimum_slack(self, point: Sequence[float]) -> float:
        """Minimum distance from a point to any bounding plane, in metres."""

        point = _vec3(point, "point")
        return float(np.min(self.b - self.A @ point))


def voxelize_inflated_spheres(
    spheres: Sequence[SphereObstacle],
    bounds_min: Sequence[float],
    bounds_max: Sequence[float],
    resolution: float,
    conservative_voxel_inflation: bool = True,
) -> np.ndarray:
    """Return occupied A* lattice points for inflated spherical obstacles.

    ``SphereObstacle.radius`` is assumed to already contain the desired
    configuration-space inflation. The optional half-voxel-diagonal padding
    exactly matches VoxelAStar3D's conservative sphere occupancy convention.
    """

    bounds_min = _vec3(bounds_min, "bounds_min")
    bounds_max = _vec3(bounds_max, "bounds_max")

    resolution = float(resolution)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("resolution must be finite and positive")
    if np.any(bounds_max <= bounds_min):
        raise ValueError("bounds_max must exceed bounds_min")

    # Match VoxelAStar3D's lattice alignment exactly.
    aligned_min = np.floor(bounds_min / resolution) * resolution
    aligned_max = np.ceil(bounds_max / resolution) * resolution
    origin = aligned_min

    grid_shape = (
        np.rint((aligned_max - aligned_min) / resolution).astype(int) + 1
    )

    voxel_padding = (
        0.5 * math.sqrt(3.0) * resolution
        if conservative_voxel_inflation
        else 0.0
    )

    occupied_index_sets = []

    for sphere in spheres:
        effective_radius = float(sphere.radius) + voxel_padding

        lower = np.floor(
            (sphere.center - effective_radius - origin) / resolution
        ).astype(int)
        upper = np.ceil(
            (sphere.center + effective_radius - origin) / resolution
        ).astype(int)

        lower = np.maximum(lower, 0)
        upper = np.minimum(upper, grid_shape - 1)

        if np.any(lower > upper):
            continue

        axes = [
            np.arange(lower[axis], upper[axis] + 1, dtype=int)
            for axis in range(3)
        ]

        indices = np.stack(
            np.meshgrid(*axes, indexing="ij"),
            axis=-1,
        ).reshape(-1, 3)

        points = origin + resolution * indices
        occupied = (
            np.linalg.norm(points - sphere.center[None, :], axis=1)
            <= effective_radius + 1e-12
        )

        if np.any(occupied):
            occupied_index_sets.append(indices[occupied])

    if not occupied_index_sets:
        return np.empty((0, 3), dtype=float)

    # Multiple obstacle spheres can occupy the same lattice point.
    occupied_indices = np.unique(
        np.vstack(occupied_index_sets),
        axis=0,
    )

    return origin + resolution * occupied_indices



@dataclass(frozen=True)
class _Ellipsoid3D:
    center: np.ndarray
    rotation: np.ndarray
    axes: np.ndarray


def _segment_frame(
    start: np.ndarray,
    end: np.ndarray,
) -> np.ndarray:
    """Return a local frame whose x-axis follows the path segment."""

    direction = end - start
    length = np.linalg.norm(direction)
    if length <= 1e-9:
        raise ValueError("corridor segment is too short")

    x_axis = direction / length

    # Match DecompUtil's convention: choose a transverse axis perpendicular
    # to both the segment's XY projection and the world vertical direction.
    y_axis = np.array([x_axis[1], -x_axis[0], 0.0], dtype=float)

    if np.linalg.norm(y_axis) <= 1e-9:
        # Segment is almost vertical.
        y_axis = np.array([-1.0, 0.0, 0.0], dtype=float)

    y_axis /= np.linalg.norm(y_axis)

    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)

    return np.column_stack((x_axis, y_axis, z_axis))


def _rotation_about_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)

    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def _local_bbox_halfspaces(
    start: np.ndarray,
    end: np.ndarray,
    local_bbox: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build the segment-aligned local bounding prism.

    local_bbox = [longitudinal extension, lateral half-width, vertical half-width].
    """

    local_bbox = _vec3(local_bbox, "local_bbox")
    if np.any(local_bbox <= 0.0):
        raise ValueError("local_bbox entries must all be positive")

    frame = _segment_frame(start, end)

    direction = frame[:, 0]
    lateral = frame[:, 1]
    vertical = frame[:, 2]

    longitudinal_extension = local_bbox[0]
    lateral_half_width = local_bbox[1]
    vertical_half_width = local_bbox[2]

    normals = np.vstack(
        (
            direction,
            -direction,
            lateral,
            -lateral,
            vertical,
            -vertical,
        )
    )

    plane_points = np.vstack(
        (
            end + direction * longitudinal_extension,
            start - direction * longitudinal_extension,
            start + lateral * lateral_half_width,
            start - lateral * lateral_half_width,
            start + vertical * vertical_half_width,
            start - vertical * vertical_half_width,
        )
    )

    offsets = np.einsum("ij,ij->i", normals, plane_points)

    return normals, offsets, frame


def _points_inside_halfspaces(
    points: np.ndarray,
    A: np.ndarray,
    b: np.ndarray,
    tolerance: float = 1e-9,
) -> np.ndarray:
    if points.size == 0:
        return np.empty((0, 3), dtype=float)

    points = np.asarray(points, dtype=float).reshape(-1, 3)

    inside = np.all(
        A @ points.T <= b[:, None] + tolerance,
        axis=0,
    )

    return points[inside]


def _ellipsoid_distances(
    points: np.ndarray,
    ellipsoid: _Ellipsoid3D,
) -> np.ndarray:
    """Dimensionless ellipsoid distance, with boundary at distance == 1."""

    if points.size == 0:
        return np.empty(0, dtype=float)

    local = (
        ellipsoid.rotation.T
        @ (points - ellipsoid.center).T
    ).T

    return np.linalg.norm(
        local / ellipsoid.axes[None, :],
        axis=1,
    )


def _fit_segment_ellipsoid(
    start: np.ndarray,
    end: np.ndarray,
    obstacle_points: np.ndarray,
    base_frame: np.ndarray,
) -> _Ellipsoid3D:
    """Fit a DecompUtil-style collision-free ellipsoid around one segment."""

    center = 0.5 * (start + end)
    half_length = 0.5 * np.linalg.norm(end - start)

    if half_length <= 1e-9:
        raise ValueError("cannot fit ellipsoid to a zero-length segment")

    # Long axis spans the line segment. Initially the two transverse axes
    # have the same radius.
    long_axis = half_length
    minor_axis_1 = half_length
    minor_axis_2 = half_length

    initial = _Ellipsoid3D(
        center=center,
        rotation=base_frame,
        axes=np.array(
            [long_axis, minor_axis_1, minor_axis_1],
            dtype=float,
        ),
    )

    initial_inside = obstacle_points[
        _ellipsoid_distances(obstacle_points, initial) <= 1.0 + 1e-10
    ]

    rotated_frame = base_frame.copy()
    remaining = initial_inside.copy()

    # First shrink a rotationally symmetric transverse radius. The closest
    # obstacle also establishes the transverse orientation used by stage 2.
    while remaining.shape[0] > 0:
        temporary = _Ellipsoid3D(
            center=center,
            rotation=rotated_frame,
            axes=np.array(
                [long_axis, minor_axis_1, minor_axis_1],
                dtype=float,
            ),
        )

        distances = _ellipsoid_distances(remaining, temporary)
        obstacle = remaining[int(np.argmin(distances))]

        local_base = base_frame.T @ (obstacle - center)

        roll = math.atan2(local_base[2], local_base[1])
        rotated_frame = base_frame @ _rotation_about_x(roll)

        local = rotated_frame.T @ (obstacle - center)

        denominator_squared = 1.0 - (local[0] / long_axis) ** 2
        if denominator_squared <= 1e-12:
            raise RuntimeError(
                "obstacle lies too close to a corridor-segment endpoint"
            )

        minor_axis_1 = (
            abs(local[1]) / math.sqrt(denominator_squared)
        )

        if minor_axis_1 <= 1e-9:
            raise RuntimeError(
                "corridor ellipsoid collapsed onto the path segment"
            )

        temporary = _Ellipsoid3D(
            center=center,
            rotation=rotated_frame,
            axes=np.array(
                [long_axis, minor_axis_1, minor_axis_1],
                dtype=float,
            ),
        )

        remaining = remaining[
            _ellipsoid_distances(remaining, temporary)
            < 1.0 - 1e-10
        ]

    # DecompUtil then restores the second transverse axis to its original
    # size and shrinks it independently. This produces a true 3-D ellipsoid
    # rather than forcing a circular cross-section.
    remaining = initial_inside.copy()

    second_stage = _Ellipsoid3D(
        center=center,
        rotation=rotated_frame,
        axes=np.array(
            [long_axis, minor_axis_1, minor_axis_2],
            dtype=float,
        ),
    )

    remaining = remaining[
        _ellipsoid_distances(remaining, second_stage)
        < 1.0 - 1e-10
    ]

    while remaining.shape[0] > 0:
        current = _Ellipsoid3D(
            center=center,
            rotation=rotated_frame,
            axes=np.array(
                [long_axis, minor_axis_1, minor_axis_2],
                dtype=float,
            ),
        )

        distances = _ellipsoid_distances(remaining, current)
        obstacle = remaining[int(np.argmin(distances))]

        local = rotated_frame.T @ (obstacle - center)

        denominator_squared = (
            1.0
            - (local[0] / long_axis) ** 2
            - (local[1] / minor_axis_1) ** 2
        )

        if denominator_squared <= 1e-12:
            raise RuntimeError(
                "unable to fit second transverse ellipsoid axis"
            )

        minor_axis_2 = (
            abs(local[2]) / math.sqrt(denominator_squared)
        )

        if minor_axis_2 <= 1e-9:
            raise RuntimeError(
                "corridor ellipsoid collapsed in its second transverse axis"
            )

        current = _Ellipsoid3D(
            center=center,
            rotation=rotated_frame,
            axes=np.array(
                [long_axis, minor_axis_1, minor_axis_2],
                dtype=float,
            ),
        )

        remaining = remaining[
            _ellipsoid_distances(remaining, current)
            < 1.0 - 1e-10
        ]

    return _Ellipsoid3D(
        center=center,
        rotation=rotated_frame,
        axes=np.array(
            [long_axis, minor_axis_1, minor_axis_2],
            dtype=float,
        ),
    )


def build_segment_polyhedron(
    segment_start: Sequence[float],
    segment_end: Sequence[float],
    obstacle_points: np.ndarray,
    local_bbox: Sequence[float],
) -> ConvexPolyhedron:
    """Construct one convex safe-flight-corridor cell."""

    start = _vec3(segment_start, "segment_start")
    end = _vec3(segment_end, "segment_end")

    obstacle_points = np.asarray(
        obstacle_points,
        dtype=float,
    ).reshape(-1, 3)

    bbox_A, bbox_b, frame = _local_bbox_halfspaces(
        start,
        end,
        local_bbox,
    )

    local_obstacles = _points_inside_halfspaces(
        obstacle_points,
        bbox_A,
        bbox_b,
        tolerance=1e-9,
    )

    ellipsoid = _fit_segment_ellipsoid(
        start,
        end,
        local_obstacles,
        frame,
    )

    remaining = local_obstacles.copy()

    separating_normals = []
    separating_offsets = []

    while remaining.shape[0] > 0:
        distances = _ellipsoid_distances(
            remaining,
            ellipsoid,
        )
        obstacle = remaining[int(np.argmin(distances))]

        local = (
            ellipsoid.rotation.T
            @ (obstacle - ellipsoid.center)
        )

        # Gradient of the ellipsoid quadratic form at the obstacle.
        normal_local = local / (ellipsoid.axes ** 2)
        normal = ellipsoid.rotation @ normal_local

        normal_length = np.linalg.norm(normal)
        if normal_length <= 1e-12:
            raise RuntimeError(
                "failed to construct an obstacle separating plane"
            )

        normal /= normal_length
        offset = float(normal @ obstacle)

        separating_normals.append(normal)
        separating_offsets.append(offset)

        # Keep obstacle points that still lie on the corridor side of this
        # separating plane.
        signed_distance = remaining @ normal - offset

        new_remaining = remaining[
            signed_distance < -1e-10
        ]

        if new_remaining.shape[0] >= remaining.shape[0]:
            raise RuntimeError(
                "convex decomposition failed to eliminate an obstacle point"
            )

        remaining = new_remaining

    if separating_normals:
        A = np.vstack(
            (
                np.asarray(separating_normals),
                bbox_A,
            )
        )
        b = np.concatenate(
            (
                np.asarray(separating_offsets),
                bbox_b,
            )
        )
    else:
        # Empty local obstacle region: the oriented local box itself is a
        # perfectly valid convex corridor cell.
        A = bbox_A
        b = bbox_b

    polyhedron = ConvexPolyhedron(
        A=A,
        b=b,
        segment_start=start,
        segment_end=end,
    )

    if not polyhedron.contains(start, tolerance=1e-7):
        raise RuntimeError(
            "generated corridor does not contain its segment start"
        )

    if not polyhedron.contains(end, tolerance=1e-7):
        raise RuntimeError(
            "generated corridor does not contain its segment end"
        )

    return polyhedron


def build_convex_corridor(
    path: np.ndarray,
    obstacle_points: np.ndarray,
    local_bbox: Sequence[float],
) -> tuple[ConvexPolyhedron, ...]:
    """Build one convex corridor cell around each path segment."""

    path = np.asarray(path, dtype=float)

    if path.ndim != 2 or path.shape[1] != 3:
        raise ValueError("path must have shape (N, 3)")
    if path.shape[0] < 2:
        raise ValueError("path must contain at least two points")
    if not np.all(np.isfinite(path)):
        raise ValueError("path must contain only finite values")

    obstacle_points = np.asarray(
        obstacle_points,
        dtype=float,
    ).reshape(-1, 3)

    cells = []

    for index in range(path.shape[0] - 1):
        start = path[index]
        end = path[index + 1]

        if np.linalg.norm(end - start) <= 1e-9:
            continue

        cells.append(
            build_segment_polyhedron(
                start,
                end,
                obstacle_points,
                local_bbox,
            )
        )

    if not cells:
        raise RuntimeError("path contains no non-zero-length segments")

    return tuple(cells)

def corridor_overlap_margins(
    path: np.ndarray,
    corridor: Sequence[ConvexPolyhedron],
) -> np.ndarray:
    """Guaranteed overlap-ball radius at each internal path waypoint."""

    path = np.asarray(path, dtype=float)

    if path.ndim != 2 or path.shape[1] != 3:
        raise ValueError("path must have shape (N, 3)")

    if len(corridor) <= 1:
        return np.empty(0, dtype=float)

    if path.shape[0] != len(corridor) + 1:
        raise ValueError(
            "path/corridor mismatch: expected one corridor cell per segment"
        )

    margins = []

    for index in range(len(corridor) - 1):
        waypoint = path[index + 1]

        previous_slack = corridor[index].minimum_slack(waypoint)
        next_slack = corridor[index + 1].minimum_slack(waypoint)

        margins.append(
            min(previous_slack, next_slack)
        )

    return np.asarray(margins, dtype=float)

def corridor_segment_containment_margins(
    corridor: Sequence[ConvexPolyhedron],
) -> np.ndarray:
    """Minimum endpoint slack for each owning A* segment."""

    margins = []

    for cell in corridor:
        start_margin = cell.minimum_slack(
            cell.segment_start
        )
        end_margin = cell.minimum_slack(
            cell.segment_end
        )

        margins.append(
            min(start_margin, end_margin)
        )

    return np.asarray(margins, dtype=float)

def corridor_point_exclusion_margins(
    corridor: Sequence[ConvexPolyhedron],
    obstacle_points: np.ndarray,
    batch_size: int = 8192,
) -> np.ndarray:
    """Check that obstacle points lie outside every corridor cell.

    Positive margin means every obstacle point is separated from the cell.
    Zero means tangency to a corridor face.
    Negative means at least one obstacle point lies inside the cell.
    """

    obstacle_points = np.asarray(
        obstacle_points,
        dtype=float,
    ).reshape(-1, 3)

    if len(corridor) == 0:
        return np.empty(0, dtype=float)

    if obstacle_points.shape[0] == 0:
        return np.full(
            len(corridor),
            np.inf,
            dtype=float,
        )

    batch_size = max(1, int(batch_size))
    cell_margins = []

    for cell in corridor:
        worst_margin = float("inf")

        for start in range(
            0,
            obstacle_points.shape[0],
            batch_size,
        ):
            batch = obstacle_points[
                start : start + batch_size
            ]

            signed_distance = (
                batch @ cell.A.T
                - cell.b[None, :]
            )

            # A point is outside the convex cell when at least one
            # half-space inequality is violated.
            per_point_margin = np.max(
                signed_distance,
                axis=1,
            )

            worst_margin = min(
                worst_margin,
                float(np.min(per_point_margin)),
            )

        cell_margins.append(worst_margin)

    return np.asarray(cell_margins, dtype=float)


def corridor_voxel_exclusion_margins(
    corridor: Sequence[ConvexPolyhedron],
    occupied_points: np.ndarray,
    voxel_resolution: float,
    batch_size: int = 8192,
) -> np.ndarray:
    """Check that complete occupied voxel cubes lie outside each corridor cell.

    Positive margin means every occupied cube is separated from the cell.
    Zero means tangency.
    Negative means the cube-volume exclusion is not satisfied.
    """

    occupied_points = np.asarray(
        occupied_points,
        dtype=float,
    ).reshape(-1, 3)

    voxel_resolution = float(voxel_resolution)

    if (
        not math.isfinite(voxel_resolution)
        or voxel_resolution <= 0.0
    ):
        raise ValueError(
            "voxel_resolution must be finite and positive"
        )

    if len(corridor) == 0:
        return np.empty(0, dtype=float)

    if occupied_points.shape[0] == 0:
        return np.full(
            len(corridor),
            np.inf,
            dtype=float,
        )

    half_width = 0.5 * voxel_resolution
    batch_size = max(1, int(batch_size))

    cell_margins = []

    for cell in corridor:
        plane_support = (
            half_width
            * np.sum(np.abs(cell.A), axis=1)
        )

        worst_margin = float("inf")

        for start in range(
            0,
            occupied_points.shape[0],
            batch_size,
        ):
            batch = occupied_points[
                start : start + batch_size
            ]

            signed_distance = (
                batch @ cell.A.T
                - cell.b[None, :]
            )

            cube_separation = (
                signed_distance
                - plane_support[None, :]
            )

            # A cube is safely outside if at least one cell plane
            # separates the entire cube.
            per_cube_margin = np.max(
                cube_separation,
                axis=1,
            )

            worst_margin = min(
                worst_margin,
                float(np.min(per_cube_margin)),
            )

        cell_margins.append(worst_margin)

    return np.asarray(cell_margins, dtype=float)

def polyhedron_vertices(
    polyhedron: ConvexPolyhedron,
    feasibility_tolerance: float = 1e-7,
    merge_tolerance: float = 1e-6,
) -> np.ndarray:
    """Recover the vertices of a bounded 3-D half-space polyhedron."""

    A = polyhedron.A
    b = polyhedron.b

    vertices = []

    for indices in combinations(range(A.shape[0]), 3):
        rows = np.asarray(indices, dtype=int)

        matrix = A[rows]
        rhs = b[rows]

        if abs(np.linalg.det(matrix)) <= 1e-10:
            continue

        vertex = np.linalg.solve(matrix, rhs)

        if np.all(
            A @ vertex <= b + feasibility_tolerance
        ):
            vertices.append(vertex)

    if not vertices:
        return np.empty((0, 3), dtype=float)

    unique_vertices = []

    for vertex in vertices:
        if not any(
            np.linalg.norm(vertex - existing) <= merge_tolerance
            for existing in unique_vertices
        ):
            unique_vertices.append(vertex)

    return np.asarray(unique_vertices, dtype=float)

def polyhedron_triangles(
    polyhedron: ConvexPolyhedron,
    plane_tolerance: float = 1e-6,
) -> np.ndarray:
    """Return triangle vertices suitable for an RViz TRIANGLE_LIST."""

    vertices = polyhedron_vertices(polyhedron)

    if vertices.shape[0] < 4:
        return np.empty((0, 3, 3), dtype=float)

    triangles = []

    for normal, offset in zip(polyhedron.A, polyhedron.b):
        distance_to_plane = np.abs(
            vertices @ normal - offset
        )

        face = vertices[
            distance_to_plane <= plane_tolerance
        ]

        if face.shape[0] < 3:
            continue

        centre = np.mean(face, axis=0)

        # Construct an orthonormal 2-D coordinate system on this plane.
        reference = (
            np.array([1.0, 0.0, 0.0])
            if abs(normal[0]) < 0.9
            else np.array([0.0, 1.0, 0.0])
        )

        axis_u = np.cross(normal, reference)
        axis_u /= np.linalg.norm(axis_u)

        axis_v = np.cross(normal, axis_u)
        axis_v /= np.linalg.norm(axis_v)

        relative = face - centre

        angles = np.arctan2(
            relative @ axis_v,
            relative @ axis_u,
        )

        face = face[np.argsort(angles)]

        # Convex face, so a fan triangulation is sufficient.
        for index in range(1, face.shape[0] - 1):
            triangles.append(
                np.vstack(
                    (
                        face[0],
                        face[index],
                        face[index + 1],
                    )
                )
            )

    if not triangles:
        return np.empty((0, 3, 3), dtype=float)

    return np.asarray(triangles, dtype=float)

