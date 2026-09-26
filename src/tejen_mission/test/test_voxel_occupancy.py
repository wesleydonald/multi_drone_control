import unittest

import numpy as np

from tejen_mission.voxel_occupancy import VoxelOccupancyGrid


class VoxelOccupancyGridTests(unittest.TestCase):
    def make_grid(self, resolution=0.10):
        return VoxelOccupancyGrid(
            resolution=resolution,
            requested_bounds_min=np.array([-0.25, -0.25, -0.25]),
            requested_bounds_max=np.array([0.25, 0.25, 0.25]),
        )

    def test_bounds_are_aligned_to_resolution(self):
        grid = self.make_grid(resolution=0.10)

        np.testing.assert_allclose(grid.bounds_min, [-0.30, -0.30, -0.30])
        np.testing.assert_allclose(grid.bounds_max, [0.30, 0.30, 0.30])
        self.assertEqual(grid.shape, (7, 7, 7))

        centre_index = grid.world_to_grid(np.zeros(3))
        np.testing.assert_array_equal(centre_index, [3, 3, 3])
        np.testing.assert_allclose(grid.grid_to_world(centre_index), np.zeros(3))

    def test_out_of_bounds_indices_are_treated_as_occupied(self):
        grid = self.make_grid()

        self.assertTrue(grid.index_is_occupied((-1, 0, 0)))
        self.assertTrue(grid.index_is_occupied((grid.shape[0], 0, 0)))
        self.assertFalse(grid.index_is_occupied(grid.world_to_grid(np.zeros(3))))

    def test_mark_sphere_includes_conservative_half_voxel_diagonal_padding(self):
        grid = self.make_grid(resolution=0.10)

        # radius=0.02 m alone would not reach an adjacent lattice centre at
        # 0.10 m, but radius + half the voxel diagonal is > 0.10 m.
        grid.mark_sphere(np.zeros(3), radius=0.02)

        self.assertTrue(grid.point_is_occupied(np.zeros(3)))
        self.assertTrue(grid.point_is_occupied(np.array([0.10, 0.0, 0.0])))
        self.assertFalse(grid.point_is_occupied(np.array([0.20, 0.0, 0.0])))

    def test_segment_is_free_detects_occupied_voxel(self):
        grid = self.make_grid(resolution=0.10)
        grid.mark_sphere(np.zeros(3), radius=0.02)

        self.assertFalse(
            grid.segment_is_free(
                np.array([-0.20, 0.0, 0.0]),
                np.array([0.20, 0.0, 0.0]),
            )
        )
        self.assertTrue(
            grid.segment_is_free(
                np.array([-0.20, 0.20, 0.20]),
                np.array([0.20, 0.20, 0.20]),
            )
        )

    def test_occupied_points_round_trip_to_occupied_indices(self):
        grid = self.make_grid(resolution=0.10)
        grid.mark_sphere(np.zeros(3), radius=0.02)

        occupied_points = grid.occupied_points()
        self.assertGreater(occupied_points.shape[0], 0)
        self.assertEqual(occupied_points.shape[1], 3)

        for point in occupied_points:
            index = grid.world_to_grid(point)
            self.assertTrue(grid.index_is_occupied(index))
            np.testing.assert_allclose(grid.grid_to_world(index), point, atol=1e-12)


if __name__ == "__main__":
    unittest.main()
