#!/usr/bin/env python3
"""Pure-Python 3D voxel A* front end for the online join planner.

The module deliberately contains no ROS dependencies.  It plans a collision-free
quad-centre path through spherical and axis-aligned-box obstacles on a regular 3D
lattice, then performs conservative line-of-sight shortcutting.  Stage-1 integration
may resample this path piecewise-linearly at low speed for testing; the later safe-
corridor stage should replace that with a genuinely smooth constrained trajectory.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import math
import time
from typing import TYPE_CHECKING, Dict, Iterable, List, Optional, Sequence, Tuple

if TYPE_CHECKING:
    from .voxel_occupancy import VoxelOccupancyGrid

import numpy as np

GridIndex = Tuple[int, int, int]


def _as_vec3(value: Sequence[float], name: str) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size != 3 or not np.all(np.isfinite(arr)):
        raise ValueError(f"{name} must contain exactly three finite numbers")
    return arr.copy()


@dataclass(frozen=True)
class SphereObstacle:
    """Sphere in the planning frame.

    ``radius`` must already include every desired configuration-space inflation
    (quad radius, tracking margin, uncertainty, etc.).  The voxel planner adds a
    small resolution-dependent term so an occupied lattice cell is conservative.
    """

    center: np.ndarray
    radius: float
    name: str = "sphere"

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", _as_vec3(self.center, "sphere center"))
        radius = float(self.radius)
        if not math.isfinite(radius) or radius < 0.0:
            raise ValueError("sphere radius must be a finite non-negative number")
        object.__setattr__(self, "radius", radius)


@dataclass(frozen=True)
class AxisAlignedBoxObstacle:
    """Axis-aligned box in the planning frame.

    ``size`` is the full x/y/z size and must already include desired
    configuration-space inflation.
    """

    center: np.ndarray
    size: np.ndarray
    name: str = "box"

    def __post_init__(self) -> None:
        object.__setattr__(self, "center", _as_vec3(self.center, "box center"))
        size = _as_vec3(self.size, "box size")
        if np.any(size < 0.0):
            raise ValueError("box size components must be non-negative")
        object.__setattr__(self, "size", size)


@dataclass
class AStarPlanResult:
    success: bool
    reason: str
    raw_path: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=float))
    simplified_path: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=float))
    planning_time_s: float = 0.0
    expanded_nodes: int = 0
    generated_nodes: int = 0
    raw_path_length_m: float = 0.0
    simplified_path_length_m: float = 0.0
    start_requested: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    goal_requested: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    start_used: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    goal_used: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    start_adjustment_m: float = 0.0
    goal_adjustment_m: float = 0.0
    bounds_min: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    bounds_max: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=float))
    grid_shape: Tuple[int, int, int] = (0, 0, 0)


class VoxelAStar3D:
    """26-connected A* on a regular 3D lattice."""

    def __init__(
        self,
        resolution: float = 0.05,
        max_expansions: int = 150_000,
        max_planning_time_s: float = 0.12,
        heuristic_weight: float = 1.0,
        nearest_free_radius_m: float = 0.50,
        line_of_sight_step_fraction: float = 0.45,
        conservative_voxel_inflation: bool = True,
    ) -> None:
        self.resolution = float(resolution)
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("resolution must be a finite positive number")

        self.max_expansions = max(1, int(max_expansions))
        self.max_planning_time_s = max(1e-4, float(max_planning_time_s))
        # 1.0 gives ordinary Euclidean A*. Values >1 are retained only as an
        # explicit weighted-A* experiment, not as the baseline.
        self.heuristic_weight = max(1.0, float(heuristic_weight))
        self.nearest_free_radius_m = max(0.0, float(nearest_free_radius_m))
        self.line_of_sight_step_fraction = float(line_of_sight_step_fraction)
        if not 0.0 < self.line_of_sight_step_fraction <= 1.0:
            raise ValueError("line_of_sight_step_fraction must be in (0, 1]")
        self.conservative_voxel_inflation = bool(conservative_voxel_inflation)

        neighbours: List[Tuple[GridIndex, float]] = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    offset = (dx, dy, dz)
                    cost = self.resolution * math.sqrt(dx * dx + dy * dy + dz * dz)
                    neighbours.append((offset, cost))
        # Deterministic order: axis moves first, then face diagonals, then body diagonals.
        neighbours.sort(key=lambda item: (item[1], item[0]))
        self._neighbours = neighbours

        self._origin = np.zeros(3, dtype=float)
        self._bounds_min = np.zeros(3, dtype=float)
        self._bounds_max = np.zeros(3, dtype=float)
        self._grid_shape = np.ones(3, dtype=int)
        self._spheres: Tuple[SphereObstacle, ...] = tuple()
        self._boxes: Tuple[AxisAlignedBoxObstacle, ...] = tuple()
        self._occupancy_cache: Dict[GridIndex, bool] = {}
        self._precomputed_occupancy: Optional["VoxelOccupancyGrid"] = None

    @staticmethod
    def path_length(path: np.ndarray) -> float:
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[0] < 2 or path.shape[1] != 3:
            return 0.0
        return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))

    @staticmethod
    def resample_polyline(
        path: np.ndarray,
        num_samples: int,
        dt: float,
        speed: float,
    ) -> np.ndarray:
        """Sample a polyline at constant arc-length speed and hold at the end."""
        path = np.asarray(path, dtype=float)
        num_samples = max(1, int(num_samples))
        dt = max(1e-6, float(dt))
        speed = max(0.0, float(speed))
        if path.ndim != 2 or path.shape[1] != 3 or path.shape[0] == 0:
            raise ValueError("path must have shape (N, 3) with N >= 1")
        if path.shape[0] == 1 or speed <= 1e-12:
            return np.repeat(path[:1], num_samples, axis=0)

        segment_lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
        cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
        total = float(cumulative[-1])
        if total <= 1e-12:
            return np.repeat(path[:1], num_samples, axis=0)

        result = np.zeros((num_samples, 3), dtype=float)
        segment_index = 0
        for k in range(num_samples):
            distance = min(total, speed * dt * k)
            while (
                segment_index < len(segment_lengths) - 1
                and cumulative[segment_index + 1] < distance
            ):
                segment_index += 1
            length = float(segment_lengths[segment_index])
            if length <= 1e-12:
                result[k] = path[segment_index + 1]
                continue
            alpha = (distance - float(cumulative[segment_index])) / length
            alpha = max(0.0, min(1.0, alpha))
            result[k] = (
                (1.0 - alpha) * path[segment_index]
                + alpha * path[segment_index + 1]
            )
        return result

    def _configure_problem(
        self,
        bounds_min: Sequence[float],
        bounds_max: Sequence[float],
        spheres: Iterable[SphereObstacle],
        boxes: Iterable[AxisAlignedBoxObstacle],
        occupancy_grid: Optional["VoxelOccupancyGrid"] = None,
    ) -> None:
        requested_min = _as_vec3(bounds_min, "bounds_min")
        requested_max = _as_vec3(bounds_max, "bounds_max")
        if np.any(requested_max <= requested_min):
            raise ValueError("every bounds_max component must exceed bounds_min")

        # Align the lattice to resolution-sized world coordinates.  Lattice nodes
        # lie at origin + integer * resolution and include both aligned endpoints.
        self._bounds_min = np.floor(requested_min / self.resolution) * self.resolution
        self._bounds_max = np.ceil(requested_max / self.resolution) * self.resolution
        self._origin = self._bounds_min.copy()
        self._grid_shape = (
            np.rint((self._bounds_max - self._bounds_min) / self.resolution).astype(int) + 1
        )
        if np.any(self._grid_shape < 2):
            raise ValueError("planning grid is degenerate")

        self._spheres = tuple(spheres)
        self._boxes = tuple(boxes)
        self._occupancy_cache = {}
        self._precomputed_occupancy = occupancy_grid

        if occupancy_grid is not None:
            if not math.isclose(
                occupancy_grid.resolution,
                self.resolution,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise ValueError(
                    "occupancy grid resolution does not match A* resolution"
                )

            if (
                not np.allclose(
                    occupancy_grid.bounds_min,
                    self._bounds_min,
                    atol=1e-12,
                )
                or not np.allclose(
                    occupancy_grid.bounds_max,
                    self._bounds_max,
                    atol=1e-12,
                )
            ):
                raise ValueError(
                    "occupancy grid bounds do not match A* bounds"
                )

    def world_to_grid(self, point: Sequence[float]) -> GridIndex:
        p = _as_vec3(point, "point")
        idx = np.rint((p - self._origin) / self.resolution).astype(int)
        return int(idx[0]), int(idx[1]), int(idx[2])

    def grid_to_world(self, index: GridIndex) -> np.ndarray:
        return self._origin + self.resolution * np.asarray(index, dtype=float)

    def _index_in_bounds(self, index: GridIndex) -> bool:
        x, y, z = index
        return (
            0 <= x < int(self._grid_shape[0])
            and 0 <= y < int(self._grid_shape[1])
            and 0 <= z < int(self._grid_shape[2])
        )

    def _point_in_bounds(self, point: np.ndarray) -> bool:
        eps = 1e-9
        return (
            self._bounds_min[0] - eps <= point[0] <= self._bounds_max[0] + eps
            and self._bounds_min[1] - eps <= point[1] <= self._bounds_max[1] + eps
            and self._bounds_min[2] - eps <= point[2] <= self._bounds_max[2] + eps
        )

    def point_is_occupied(self, point: Sequence[float]) -> bool:
        p = _as_vec3(point, "point")
        if not self._point_in_bounds(p):
            return True

        if self._precomputed_occupancy is not None:
            return self._precomputed_occupancy.point_is_occupied(p)

        sphere_voxel_padding = (
            0.5 * math.sqrt(3.0) * self.resolution
            if self.conservative_voxel_inflation
            else 0.0
        )
        for sphere in self._spheres:
            if float(np.linalg.norm(p - sphere.center)) <= sphere.radius + sphere_voxel_padding:
                return True

        box_voxel_padding = 0.5 * self.resolution if self.conservative_voxel_inflation else 0.0
        for box in self._boxes:
            half = 0.5 * box.size + box_voxel_padding
            if bool(np.all(np.abs(p - box.center) <= half)):
                return True

        return False

    def index_is_occupied(self, index: GridIndex) -> bool:
        if not self._index_in_bounds(index):
            return True
        if self._precomputed_occupancy is not None:
            return self._precomputed_occupancy.index_is_occupied(index)
        cached = self._occupancy_cache.get(index)
        if cached is not None:
            return cached
        occupied = self.point_is_occupied(self.grid_to_world(index))
        self._occupancy_cache[index] = occupied
        return occupied

    def segment_is_free(self, start: Sequence[float], end: Sequence[float]) -> bool:
        a = _as_vec3(start, "segment start")
        b = _as_vec3(end, "segment end")
        delta = b - a
        length = float(np.linalg.norm(delta))
        if length <= 1e-12:
            return not self.point_is_occupied(a)

        step = max(1e-6, self.line_of_sight_step_fraction * self.resolution)
        samples = max(1, int(math.ceil(length / step)))
        for i in range(samples + 1):
            p = a + (i / samples) * delta
            if self.point_is_occupied(p):
                return False
        return True

    def _nearest_free_index(self, requested: GridIndex) -> Optional[GridIndex]:
        if self._index_in_bounds(requested) and not self.index_is_occupied(requested):
            return requested

        max_steps = int(math.ceil(self.nearest_free_radius_m / self.resolution))
        if max_steps <= 0:
            return None

        # Search lattice candidates by Euclidean offset distance.  This is done only
        # for start/goal recovery, so a pre-sorted local list is simple and reliable.
        candidates: List[Tuple[int, int, int, int]] = []
        for dx in range(-max_steps, max_steps + 1):
            for dy in range(-max_steps, max_steps + 1):
                for dz in range(-max_steps, max_steps + 1):
                    d2 = dx * dx + dy * dy + dz * dz
                    if d2 == 0 or math.sqrt(d2) > max_steps + 1e-9:
                        continue
                    candidates.append((d2, dx, dy, dz))
        candidates.sort()

        for _, dx, dy, dz in candidates:
            candidate = (requested[0] + dx, requested[1] + dy, requested[2] + dz)
            if self._index_in_bounds(candidate) and not self.index_is_occupied(candidate):
                return candidate
        return None

    def _transition_is_free(self, current: GridIndex, neighbour: GridIndex) -> bool:
        """Reject 26-connected diagonal corner cutting using cached cell occupancy."""
        cx, cy, cz = current
        dx = neighbour[0] - cx
        dy = neighbour[1] - cy
        dz = neighbour[2] - cz

        # Face diagonal: both axis-adjacent cells must be free.
        if dx != 0 and dy != 0:
            if self.index_is_occupied((cx + dx, cy, cz)):
                return False
            if self.index_is_occupied((cx, cy + dy, cz)):
                return False
        if dx != 0 and dz != 0:
            if self.index_is_occupied((cx + dx, cy, cz)):
                return False
            if self.index_is_occupied((cx, cy, cz + dz)):
                return False
        if dy != 0 and dz != 0:
            if self.index_is_occupied((cx, cy + dy, cz)):
                return False
            if self.index_is_occupied((cx, cy, cz + dz)):
                return False

        # Body diagonal: also require the three face-diagonal intermediate cells.
        if dx != 0 and dy != 0 and dz != 0:
            if self.index_is_occupied((cx + dx, cy + dy, cz)):
                return False
            if self.index_is_occupied((cx + dx, cy, cz + dz)):
                return False
            if self.index_is_occupied((cx, cy + dy, cz + dz)):
                return False

        # Each occupied lattice cell is conservatively inflated by half a voxel
        # diagonal, so free endpoints plus the intermediate-cell checks above are
        # sufficient for the short neighbour edge. Long line-of-sight shortcuts are
        # still densely sampled by segment_is_free().
        return True

    def simplify_path(self, path: np.ndarray) -> np.ndarray:
        path = np.asarray(path, dtype=float)
        if path.ndim != 2 or path.shape[1] != 3:
            raise ValueError("path must have shape (N, 3)")
        if path.shape[0] <= 2:
            return path.copy()

        simplified = [path[0].copy()]
        anchor = 0
        final = path.shape[0] - 1
        while anchor < final:
            next_index = anchor + 1
            # Select the farthest later point visible from the current anchor.
            for candidate in range(final, anchor, -1):
                if self.segment_is_free(path[anchor], path[candidate]):
                    next_index = candidate
                    break
            simplified.append(path[next_index].copy())
            anchor = next_index
        return np.asarray(simplified, dtype=float)

    def plan(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        bounds_min: Sequence[float],
        bounds_max: Sequence[float],
        spheres: Iterable[SphereObstacle] = (),
        boxes: Iterable[AxisAlignedBoxObstacle] = (),
        occupancy_grid: Optional["VoxelOccupancyGrid"] = None,
    ) -> AStarPlanResult:
        start_time = time.perf_counter()
        start_requested = _as_vec3(start, "start")
        goal_requested = _as_vec3(goal, "goal")

        result = AStarPlanResult(
            success=False,
            reason="not started",
            start_requested=start_requested,
            goal_requested=goal_requested,
            start_used=start_requested.copy(),
            goal_used=goal_requested.copy(),
        )

        try:
            self._configure_problem(
                bounds_min,
                bounds_max,
                spheres,
                boxes,
                occupancy_grid,
            )
        except ValueError as exc:
            result.reason = f"invalid planning problem: {exc}"
            result.planning_time_s = time.perf_counter() - start_time
            return result

        result.bounds_min = self._bounds_min.copy()
        result.bounds_max = self._bounds_max.copy()
        result.grid_shape = tuple(int(v) for v in self._grid_shape)

        requested_start_index = self.world_to_grid(start_requested)
        requested_goal_index = self.world_to_grid(goal_requested)
        start_index = self._nearest_free_index(requested_start_index)
        goal_index = self._nearest_free_index(requested_goal_index)

        if start_index is None:
            result.reason = "no free lattice node near start"
            result.planning_time_s = time.perf_counter() - start_time
            return result
        if goal_index is None:
            result.reason = "no free lattice node near goal"
            result.planning_time_s = time.perf_counter() - start_time
            return result

        start_used = self.grid_to_world(start_index)
        goal_used = self.grid_to_world(goal_index)
        result.start_used = start_used.copy()
        result.goal_used = goal_used.copy()
        result.start_adjustment_m = float(np.linalg.norm(start_used - start_requested))
        result.goal_adjustment_m = float(np.linalg.norm(goal_used - goal_requested))

        if start_index == goal_index:
            raw = np.vstack((start_used,))
            result.success = True
            result.reason = "start and goal map to the same free lattice node"
            result.raw_path = raw
            result.simplified_path = raw.copy()
            result.planning_time_s = time.perf_counter() - start_time
            return result

        def heuristic(index: GridIndex) -> float:
            dx = index[0] - goal_index[0]
            dy = index[1] - goal_index[1]
            dz = index[2] - goal_index[2]
            return self.heuristic_weight * self.resolution * math.sqrt(dx * dx + dy * dy + dz * dz)

        open_heap: List[Tuple[float, float, int, GridIndex]] = []
        counter = 0
        g_score: Dict[GridIndex, float] = {start_index: 0.0}
        came_from: Dict[GridIndex, GridIndex] = {}
        closed: set[GridIndex] = set()
        heapq.heappush(open_heap, (heuristic(start_index), 0.0, counter, start_index))
        generated_nodes = 1
        expanded_nodes = 0
        found = False

        while open_heap:
            _, queued_g, _, current = heapq.heappop(open_heap)
            current_best_g = g_score.get(current)
            if current_best_g is None or queued_g > current_best_g + 1e-12:
                continue
            if current in closed:
                continue
            closed.add(current)
            expanded_nodes += 1

            if current == goal_index:
                found = True
                break
            if expanded_nodes >= self.max_expansions:
                result.reason = f"maximum expansion count reached ({self.max_expansions})"
                break
            if time.perf_counter() - start_time >= self.max_planning_time_s:
                result.reason = (
                    f"planning time budget reached ({1000.0 * self.max_planning_time_s:.1f} ms)"
                )
                break

            for offset, move_cost in self._neighbours:
                neighbour = (
                    current[0] + offset[0],
                    current[1] + offset[1],
                    current[2] + offset[2],
                )
                if neighbour in closed or self.index_is_occupied(neighbour):
                    continue
                if not self._transition_is_free(current, neighbour):
                    continue

                tentative_g = current_best_g + move_cost
                if tentative_g + 1e-12 >= g_score.get(neighbour, float("inf")):
                    continue
                came_from[neighbour] = current
                g_score[neighbour] = tentative_g
                counter += 1
                generated_nodes += 1
                heapq.heappush(
                    open_heap,
                    (tentative_g + heuristic(neighbour), tentative_g, counter, neighbour),
                )

        result.expanded_nodes = expanded_nodes
        result.generated_nodes = generated_nodes

        if not found:
            if result.reason == "not started":
                result.reason = "open set exhausted; no path exists in the local grid"
            result.planning_time_s = time.perf_counter() - start_time
            return result

        indices = [goal_index]
        while indices[-1] != start_index:
            indices.append(came_from[indices[-1]])
        indices.reverse()
        raw = np.vstack([self.grid_to_world(index) for index in indices])

        # Preserve exact requested endpoints only when they are themselves free and
        # connect safely to the snapped lattice path.  If start/goal were recovered
        # from inside an obstacle, keep the adjusted lattice point instead.
        if (
            result.start_adjustment_m <= 0.75 * self.resolution
            and not self.point_is_occupied(start_requested)
            and (raw.shape[0] == 1 or self.segment_is_free(start_requested, raw[1]))
        ):
            raw[0] = start_requested
        if (
            result.goal_adjustment_m <= 0.75 * self.resolution
            and not self.point_is_occupied(goal_requested)
            and (raw.shape[0] == 1 or self.segment_is_free(raw[-2], goal_requested))
        ):
            raw[-1] = goal_requested

        simplified = self.simplify_path(raw)
        result.success = True
        result.reason = "path found"
        if result.start_adjustment_m > 0.75 * self.resolution:
            result.reason += f"; start adjusted {result.start_adjustment_m:.3f} m"
        if result.goal_adjustment_m > 0.75 * self.resolution:
            result.reason += f"; goal adjusted {result.goal_adjustment_m:.3f} m"
        result.raw_path = raw
        result.simplified_path = simplified
        result.raw_path_length_m = self.path_length(raw)
        result.simplified_path_length_m = self.path_length(simplified)
        result.planning_time_s = time.perf_counter() - start_time
        return result
