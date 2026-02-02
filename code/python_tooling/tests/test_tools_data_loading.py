import unittest

from dashboard.tools.data_loading import (
    refresh_sensor_list,
)
from database.types import SensorType


class TestDashboardToolsDataLoading(unittest.TestCase):
    def test_refresh_sensor_list_bad_inputs(self):
        # The input metadata is None
        sensor_names, default_value = refresh_sensor_list(None, SensorType.Camera)
        self.assertEqual(sensor_names, [])
        self.assertEqual(default_value, "")

        # Requested sensor_type not present in metadata.statistics
        statistics = {SensorType.Imu: None}
        sensor_names, default_value = refresh_sensor_list(
            [statistics, None], SensorType.Camera
        )
        self.assertEqual(sensor_names, [])
        self.assertEqual(default_value, "")

    def test_refresh_sensor_list(self):
        statistics = ({SensorType.Camera: {}},)
        sensor_names, default_value = refresh_sensor_list(
            [statistics, None], SensorType.Camera
        )
        self.assertEqual(sensor_names, [])
        self.assertEqual(default_value, "")

        statistics = {SensorType.Camera: {"/sensor_1": None, "/sensor_2": None}}
        sensor_names, default_value = refresh_sensor_list(
            [statistics, None], SensorType.Camera
        )
        self.assertEqual(sensor_names, ["/sensor_1", "/sensor_2"])
        self.assertEqual(default_value, "/sensor_1")
