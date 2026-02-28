import os
import unittest
from pathlib import Path

from dashboard.tools.data_loading import (
    load_database,
    refresh_database_list,
    refresh_sensor_list,
)
from database.calculate_metadata import count_data, reference_timestamps
from database.data_formatting import load_data
from database.types import SensorType


class TestDataLoading(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_refresh_database_list(self):
        db_dir = Path(self.db_path).parent
        database_list, first_database = refresh_database_list(db_dir)

        # NOTE(Jack): Because we run this locally and remotely, and with other databases possibly in the test_data
        # folder, we make this test as unrestrictive as possible, only checking that checked in test database is present
        # and that the first database exists (not exactly which database it is).
        self.assertTrue(
            any(
                item["label"] == "dataset-calib-imu4_512_16.db3"
                for item in database_list
            )
        )
        self.assertTrue(os.path.isfile(first_database))

    def test_refresh_database_list_adversarial(self):
        db_paths = refresh_database_list(None)
        self.assertEqual(db_paths, ([], ""))

        db_paths = refresh_database_list("directory/that/does/not/exist/")
        self.assertEqual(db_paths, ([], ""))

    def test_load_database(self):
        raw_data, metadata, timestamps = load_database(self.db_path)

        gt_metadata = {
            "/cam0/image_raw": {
                "type": SensorType.Camera,
                "measurements": {"images": 879, "targets": 879},
            },
            "/cam1/image_raw": {
                "type": SensorType.Camera,
                "measurements": {"images": 879, "targets": 879},
            },
            "/imu0": {"type": SensorType.Imu, "measurements": 8770},
        }

        self.assertEqual(count_data(raw_data), gt_metadata)
        self.assertEqual(metadata, gt_metadata)
        self.assertEqual(timestamps, reference_timestamps(raw_data))

    def test_load_database_adversarial(self):
        result = load_database(None)
        self.assertEqual(result, (None, None, None))

        result = load_database("file/that/does/not/exist.db3")
        self.assertEqual(result, (None, None, None))

    def test_refresh_sensor_list(self):
        data = load_data(self.db_path)
        metadata = count_data(data)

        sensor_list, first_sensor = refresh_sensor_list(metadata)

        gt_sensor_list = [
            {
                "label": "/cam0/image_raw (SensorType.Camera)",
                "value": "/cam0/image_raw",
            },
            {
                "label": "/cam1/image_raw (SensorType.Camera)",
                "value": "/cam1/image_raw",
            },
            {"label": "/imu0 (SensorType.Imu)", "value": "/imu0"},
        ]
        self.assertEqual(sensor_list, gt_sensor_list)
        self.assertEqual(first_sensor, gt_sensor_list[0]["value"])

    def test_refresh_sensor_list_adversarial(self):
        sensor_data = refresh_sensor_list(None)
        self.assertEqual(sensor_data, ([], ""))

        sensor_data = refresh_sensor_list({})
        self.assertEqual(sensor_data, ([], ""))
