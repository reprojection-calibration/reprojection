import unittest

from dash.exceptions import PreventUpdate
from reference_data import FullData as FD
from reference_data import InvalidData as ID
from reference_data import SkeletonData as SD

from dashboard.callbacks.statistics import build_sensor_statistics_div
from database.types import SensorType


# TODO(Jack): Why are we still raising an error here with this method instead of just returning blank?
class TestDashboardCallbacksStatistics(unittest.TestCase):
    def test_build_sensor_statistics_div_invalid_data(self):
        self.assertRaises(
            PreventUpdate,
            build_sensor_statistics_div,
            "",
            ID.metadata,
            SensorType.Camera,
        )

    def test_build_sensor_statistics_div_skeleton_data(self):
        self.assertRaises(
            PreventUpdate,
            build_sensor_statistics_div,
            "",
            SD.metadata,
            SensorType.Camera,
        )

    def test_build_sensor_statistics_div(self):
        # We add another statistic so that we can test the two mains behaviors of the function.
        test_data = FD()
        test_data.statistics[SensorType.Camera]["/cam0/image_raw"]["statistic_1"] = 0
        output = build_sensor_statistics_div(
            "/cam0/image_raw",
            test_data.metadata,
            SensorType.Camera,
        )

        self.assertEqual(len(output), 2)

        def assert_statistic_row_element(div, color, value, label):
            div_colored_box = div.children[0]
            self.assertEqual(div_colored_box.style["backgroundColor"], color)

            div_counter = div.children[1]
            self.assertEqual(div_counter.children, value)

            div_label = div.children[2]
            self.assertEqual(div_label.children, label)

        # The total frame count is non-zero so it should display a green box
        div_statistic_1 = output[0]
        self.assertEqual(len(div_statistic_1.children), 3)
        assert_statistic_row_element(div_statistic_1, "green", 1, "total_frames")

        # The statistic_1 has a value of zero so it should display a red box
        div_statistic_2 = output[1]
        self.assertEqual(len(div_statistic_2.children), 3)
        assert_statistic_row_element(div_statistic_2, "red", 0, "statistic_1")
