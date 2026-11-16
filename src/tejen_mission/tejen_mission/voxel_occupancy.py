"""ROS-independent precomputed voxel occupancy grid."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Sequence, Tuple

import numpy as np


GridIndex = Tuple[int, int, int]


def _vec3(value: Sequence[float], name: str) -> np.ndarray:
    """Initialises a np.ndarray of 3 elements if the input is made of 3 finite values."""
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (3,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain exactly three finite values")
    return array.copy()


@dataclass
class VoxelOccupancyGrid:
    """The class that defines the 3D occupancy map"""
    # resolution and bounds of the grid - resolution probably in m and bounds in m too
    resolution: float
    requested_bounds_min: np.ndarray
    requested_bounds_max: np.ndarray

    bounds_min: np.ndarray = field(init=False)
    bounds_max: np.ndarray = field(init=False)
    shape: Tuple[int, int, int] = field(init=False)
    occupied: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.resolution = float(self.resolution)
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("resolution must be finite and positive")

        requested_min = _vec3(
            self.requested_bounds_min,
            "requested_bounds_min",
        )
        requested_max = _vec3(
            self.requested_bounds_max,
            "requested_bounds_max",
        )

        # error checking of bounds
        if np.any(requested_max <= requested_min):
            raise ValueError("planning bounds are invalid")

        # the bounds that are actually set need to be multiples of the resolution
        self.bounds_min = (
            np.floor(requested_min / self.resolution)
            * self.resolution
        )
        self.bounds_max = (
            np.ceil(requested_max / self.resolution)
            * self.resolution
        )

        grid_shape = (
            np.rint(
                (self.bounds_max - self.bounds_min)
                / self.resolution
            ).astype(int)
            + 1
        )

        self.shape = tuple(int(value) for value in grid_shape)
        self.occupied = np.zeros(self.shape, dtype=bool)

    # convert a point in world to a point in the grid
    def world_to_grid(self, point: Sequence[float]) -> GridIndex:
        point = _vec3(point, "point")
        index = np.rint(
            (point - self.bounds_min) / self.resolution
        ).astype(int)

        return int(index[0]), int(index[1]), int(index[2])

    # convert a point in the grid to a point in the world
    def grid_to_world(self, index: GridIndex) -> np.ndarray:
        return (
            self.bounds_min
            + self.resolution * np.asarray(index, dtype=float)
        )

    # return true if the index is in the bounds of the shape
    def index_in_bounds(self, index: GridIndex) -> bool:
        return all(
            0 <= index[axis] < self.shape[axis]
            for axis in range(3)
        )

    # return true if the valid index is occupied or true if it's out of bounds
    def index_is_occupied(self, index: GridIndex) -> bool:
        if not self.index_in_bounds(index):
            return True

        return bool(self.occupied[index])

    # a different way of calling index_is_occupied from the world perspective
    def point_is_occupied(self, point: Sequence[float]) -> bool:
        return self.index_is_occupied(
            self.world_to_grid(point)
        )

    def segment_is_free(
        self,
        start: Sequence[float],
        end: Sequence[float],
        step_fraction: float = 0.45,
    ) -> bool:
        """check if the segment defined by 2 3D points and a floating point does not
        intersect any occupied points"""
        start = _vec3(start, "segment start")
        end = _vec3(end, "segment end")

        length = float(np.linalg.norm(end - start))

        if length <= 1e-12:
            return not self.point_is_occupied(start)

        step = max(
            1e-6,
            float(step_fraction) * self.resolution,
        )
        samples = max(1, int(math.ceil(length / step)))

        for index in range(samples + 1):
            alpha = index / samples
            point = (1.0 - alpha) * start + alpha * end

            if self.point_is_occupied(point):
                return False

        return True

    def mark_sphere(
        self,
        centre: Sequence[float],
        radius: float,
    ) -> None:
        """Mark a spherical section off in the occupancy grid"""
        centre = _vec3(centre, "sphere centre")
        radius = float(radius)

        if radius < 0.0 or not math.isfinite(radius):
            raise ValueError("sphere radius must be finite and non-negative")

        # Mark a voxel occupied if its cell can intersect the sphere.
        effective_radius = (
            radius
            + 0.5 * math.sqrt(3.0) * self.resolution
        )

        lower = np.floor(
            (centre - effective_radius - self.bounds_min)
            / self.resolution
        ).astype(int)

        upper = np.ceil(
            (centre + effective_radius - self.bounds_min)
            / self.resolution
        ).astype(int)

        lower = np.maximum(lower, 0)
        upper = np.minimum(
            upper,
            np.asarray(self.shape) - 1,
        )

        if np.any(lower > upper):
            return

        axes = [
            np.arange(lower[axis], upper[axis] + 1)
            for axis in range(3)
        ]

        indices = np.stack(
            np.meshgrid(*axes, indexing="ij"),
            axis=-1,
        ).reshape(-1, 3)

        points = (
            self.bounds_min
            + self.resolution * indices
        )

        inside = (
            np.linalg.norm(
                points - centre[None, :],
                axis=1,
            )
            <= effective_radius + 1e-12
        )

        occupied_indices = indices[inside]

        self.occupied[
            occupied_indices[:, 0],
            occupied_indices[:, 1],
            occupied_indices[:, 2],
        ] = True

    def occupied_points(self) -> np.ndarray:
        """Return the full list of occupied points in the world frame"""
        indices = np.argwhere(self.occupied)

        if indices.shape[0] == 0:
            return np.empty((0, 3), dtype=float)

        return (
            self.bounds_min[None, :]
            + self.resolution * indices
        )