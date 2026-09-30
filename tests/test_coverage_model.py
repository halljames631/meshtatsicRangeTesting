"""Tests for RF model calculations and analyzer sweep parsers."""

from __future__ import annotations

import unittest
from unittest.mock import Mock, patch

from radio_range_monitor.coverage_model import (
    effective_receiver_sensitivity_dbm,
    fetch_elevation_profile,
    generate_coverage_grid,
    hata_urban_path_loss_db,
    haversine_m,
    parse_nanovna_sweep,
    parse_tinysa_sweep,
    swr_mismatch_loss_db,
)


class DistanceTests(unittest.TestCase):
    def test_haversine_returns_great_circle_distance_in_metres(self) -> None:
        self.assertAlmostEqual(haversine_m(0, 0, 0, 1), 111_194.93, delta=1)

    def test_identical_points_have_zero_distance(self) -> None:
        self.assertEqual(haversine_m(47.0, -122.0, 47.0, -122.0), 0)


class ElevationProfileTests(unittest.TestCase):
    @patch.dict("os.environ", {}, clear=True)
    @patch("radio_range_monitor.coverage_model.requests.get")
    def test_uses_open_meteo_for_default_elevation_source(self, get: Mock) -> None:
        get.return_value = Mock(
            json=Mock(return_value={"elevation": [100, 110, 120]}),
            raise_for_status=Mock(),
        )

        distances, elevations, adjusted = fetch_elevation_profile(
            47.0, -122.0, 47.01, -122.01, samples=3
        )

        self.assertEqual(len(distances), 3)
        self.assertEqual(elevations.tolist(), [100.0, 110.0, 120.0])
        self.assertEqual(len(adjusted), 3)
        self.assertEqual(
            get.call_args.args[0], "https://api.open-meteo.com/v1/elevation"
        )
        self.assertEqual(len(get.call_args.kwargs["params"]["latitude"].split(",")), 3)
        self.assertEqual(len(get.call_args.kwargs["params"]["longitude"].split(",")), 3)

    @patch.dict("os.environ", {"ELEVATION_API_URL": "https://elevation.example/lookup"})
    @patch("radio_range_monitor.coverage_model.requests.post")
    def test_keeps_open_elevation_compatible_custom_endpoint(self, post: Mock) -> None:
        post.return_value = Mock(
            json=Mock(
                return_value={
                    "results": [
                        {"elevation": 100},
                        {"elevation": 110},
                        {"elevation": 120},
                    ]
                }
            ),
            raise_for_status=Mock(),
        )

        _, elevations, _ = fetch_elevation_profile(
            47.0, -122.0, 47.01, -122.01, samples=3
        )

        self.assertEqual(elevations.tolist(), [100.0, 110.0, 120.0])
        self.assertEqual(post.call_args.args[0], "https://elevation.example/lookup")


class LinkBudgetTests(unittest.TestCase):
    def test_swr_mismatch_loss_is_zero_for_matched_feed(self) -> None:
        self.assertAlmostEqual(swr_mismatch_loss_db(1.0), 0)

    def test_invalid_swr_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            swr_mismatch_loss_db(0.9)

    def test_noise_floor_and_required_snr_raise_effective_sensitivity(self) -> None:
        threshold = effective_receiver_sensitivity_dbm(-95, -120, -7.5)
        self.assertEqual(threshold, -102.5)

    def test_hata_rejects_frequency_outside_model_range(self) -> None:
        with self.assertRaises(ValueError):
            hata_urban_path_loss_db(2400, 30, 2, 2.0)

    def test_coverage_grid_returns_points_and_signal_classes(self) -> None:
        result = generate_coverage_grid(
            47.0,
            -122.0,
            915.0,
            30.0,
            2.0,
            2.0,
            30.0,
            2.0,
            1.2,
            -110.0,
            -120.0,
            -7.5,
            max_radius_m=1_000,
            spacing_m=250,
        )
        self.assertEqual(result["grid_points"], 49)
        self.assertGreater(len(result["polygons"]), 0)
        self.assertEqual(result["sensitivity_dbm"], -117.5)


class AnalyzerParserTests(unittest.TestCase):
    def test_tinysa_parser_ignores_rows_outside_requested_band(self) -> None:
        response = "914000000,-108.0\n915000000,-105.2\n"
        self.assertEqual(
            parse_tinysa_sweep(response, 915_000_000, 920_000_000),
            [-105.2],
        )

    def test_nanovna_parser_returns_s11_components(self) -> None:
        response = "915000000,0.2,0.1\n"
        self.assertEqual(
            parse_nanovna_sweep(response, 910_000_000, 920_000_000),
            [(915_000_000.0, 0.2, 0.1)],
        )

    def test_parsers_reject_non_numeric_sweep_data(self) -> None:
        self.assertEqual(parse_tinysa_sweep("sweep failed", 1, 2), [])
        self.assertEqual(parse_nanovna_sweep("sweep failed", 1, 2), [])


if __name__ == "__main__":
    unittest.main()
