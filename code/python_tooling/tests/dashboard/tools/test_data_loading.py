import unittest
import os

from dashboard.tools.data_loading import refresh_sensor_list
from database.types import SensorType

from database.data_formatting import load_data
from database.calculate_metadata import count_data


class TestDataLoading(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_refresh_sensor_list(self):
        data = load_data(self.db_path)
        metadata = count_data(data)

        sensor_list, first_sensor = refresh_sensor_list(metadata)

        gt_sensor_list = ['/cam0/image_raw (Camera)', '/cam1/image_raw (Camera)', '/imu0 (Imu)']
        self.assertEqual(sensor_list, gt_sensor_list)
        self.assertEqual(first_sensor, gt_sensor_list[0])

    def test_refresh_sensor_list_adversarial(self):
        metadata = refresh_sensor_list(None)
        self.assertEqual(metadata, ([], ''))

        metadata = refresh_sensor_list({})
        self.assertEqual(metadata, ([], ''))

