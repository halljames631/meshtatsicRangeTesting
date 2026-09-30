"""Tests for shared legacy RF utilities."""

from __future__ import annotations

import unittest

from radio_range_monitor.legacy.backend import (
    coordinates_to_world_pixel,
    free_space_path_loss_db,
    haversine_m,
    world_pixel_to_coordinates,
)


class LegacyRFUtilityTests(unittest.TestCase):
    def test_world_pixel_projection_round_trips_coordinates(self) -> None:
        latitude, longitude = 47.6062, -122.3321
        pixel = coordinates_to_world_pixel(latitude, longitude, zoom=10)
        result = world_pixel_to_coordinates(*pixel, zoom=10)
        self.assertAlmostEqual(result[0], latitude, places=5)
        self.assertAlmostEqual(result[1], longitude, places=5)

    def test_free_space_path_loss_rejects_non_positive_inputs(self) -> None:
        with self.assertRaises(ValueError):
            free_space_path_loss_db(915, 0)

    def test_legacy_haversine_matches_one_degree_at_equator(self) -> None:
        self.assertAlmostEqual(haversine_m(0, 0, 0, 1), 111_194.93, delta=1)


if __name__ == "__main__":
    unittest.main()
