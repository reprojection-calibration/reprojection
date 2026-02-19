import unittest

from dashboard_reference_data import full_data, invalid_data, skeleton_data

from dashboard.tools.data_loading import refresh_sensor_list
from database.types import SensorType


class TestDashboardToolsDataLoading(unittest.TestCase):
    def test_refresh_sensor_list_invalid_data(self):
        test_data = invalid_data()

        sensor_names, default_value = refresh_sensor_list(
            test_data.metadata, SensorType.Camera
        )
        self.assertEqual(sensor_names, [])
        self.assertEqual(default_value, "")

    def test_refresh_sensor_list_skeleton_data(self):
        test_data = skeleton_data()

        sensor_names, default_value = refresh_sensor_list(
            test_data.metadata, SensorType.Camera
        )
        self.assertEqual(sensor_names, [])
        self.assertEqual(default_value, "")

    def test_refresh_sensor_list(self):
        test_data = full_data()

        sensor_names, default_value = refresh_sensor_list(
            test_data.metadata, SensorType.Camera
        )
        self.assertEqual(sensor_names, ["/cam0/image_raw"])
        self.assertEqual(default_value, "/cam0/image_raw")
