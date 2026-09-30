"""Tests for map-point and terrain-profile UI state transitions."""

from __future__ import annotations

import unittest
from unittest.mock import Mock, patch

from matplotlib.figure import Figure

from radio_range_monitor.coverage_model import TELEMETRY
from radio_range_monitor.dashboard import RFDesktopApp


class TestVariable:
    def __init__(self, value: str = "") -> None:
        self.value = value

    def get(self) -> str:
        return self.value

    def set(self, value: str) -> None:
        self.value = value


class DashboardStateTests(unittest.TestCase):
    def make_app(self) -> RFDesktopApp:
        app = object.__new__(RFDesktopApp)
        app._map_next_point_is_start = True
        app.terrain_generation = 1
        app.coverage_generation = 0
        app._input_signature = None
        app._coverage_after_id = None
        app.root = Mock()
        app.home_marker = Mock()
        app.target_marker = Mock()
        app.link_path = Mock()
        app.link_los_blocked = True
        app.coverage_polygons = []
        app.home_lat_var = TestVariable("1")
        app.home_lon_var = TestVariable("2")
        app.target_lat_var = TestVariable("3")
        app.target_lon_var = TestVariable("4")
        app.coverage_status_var = TestVariable()
        app.terrain_status_var = TestVariable()
        app.map_link_status_var = TestVariable()
        app.point_selection_status_var = TestVariable()
        app.map_link_status_label = Mock()
        app.figure = Figure()
        app.axes = app.figure.add_subplot(111)
        app.figure_canvas = Mock()
        return app

    def test_start_point_discards_previous_route_and_profile(self) -> None:
        app = self.make_app()
        old_target_marker = app.target_marker
        old_link_path = app.link_path

        with patch.object(RFDesktopApp, "_map_set_home", autospec=True):
            RFDesktopApp._map_select_profile_point(app, (47.0, -122.0))

        self.assertEqual(app.terrain_generation, 2)
        self.assertFalse(app._map_next_point_is_start)
        old_target_marker.delete.assert_called_once_with()
        old_link_path.delete.assert_called_once_with()
        self.assertIsNone(app.target_marker)
        self.assertIsNone(app.link_path)
        self.assertEqual(app.target_lat_var.get(), "")
        self.assertEqual(app.target_lon_var.get(), "")
        self.assertIsNone(app.link_los_blocked)
        self.assertIn("select an end point", app.map_link_status_var.get())
        self.assertIn("Start point selected", app.terrain_status_var.get())
        self.assertIn("Step 2 of 2", app.point_selection_status_var.get())

    def test_second_map_click_starts_profile_and_shows_progress_state(self) -> None:
        app = self.make_app()
        app._map_next_point_is_start = False

        with (
            patch.object(RFDesktopApp, "_map_set_target", autospec=True),
            patch.object(RFDesktopApp, "fetch_terrain", autospec=True) as fetch,
        ):
            RFDesktopApp._map_select_profile_point(app, (47.1, -122.1))

        fetch.assert_called_once_with(app)
        self.assertTrue(app._map_next_point_is_start)
        self.assertIn("requesting", app.point_selection_status_var.get())

    def test_modern_scale_keeps_configured_value_increment(self) -> None:
        app = self.make_app()
        app.schedule_coverage_update = Mock()
        value_var = Mock()
        scale_var = Mock()

        RFDesktopApp._scale_value_changed(
            app,
            "12.8",
            value_var,
            scale_var,
            minimum=0,
            resolution=1,
        )

        scale_var.set.assert_called_once_with(13)
        value_var.set.assert_called_once_with("13")
        app.schedule_coverage_update.assert_called_once_with()

    def test_reset_keeps_reference_to_active_terrain_request(self) -> None:
        app = self.make_app()
        active_thread = Mock()
        app.terrain_thread = active_thread

        with patch.dict(TELEMETRY):
            RFDesktopApp.reset_profile_points(app)

        self.assertIs(app.terrain_thread, active_thread)
        self.assertEqual(app.terrain_generation, 2)
        self.assertTrue(app._map_next_point_is_start)
        self.assertEqual(app.home_lat_var.get(), "")
        self.assertEqual(app.target_lat_var.get(), "")


if __name__ == "__main__":
    unittest.main()
