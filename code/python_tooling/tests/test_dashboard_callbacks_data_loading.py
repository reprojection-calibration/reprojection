import os
import unittest
from pathlib import Path

from dashboard.callbacks.data_loading import (
    get_sensor_names,
    load_database_to_store,
    refresh_database_list,
)
from database.types import SensorType


class TestDashboardCallbacksDataLoading(unittest.TestCase):
    # NOTE(Jack): See note in TestDatabaseCameraCalibrationData
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_refresh_database_list(self):
        # The value of n_clicks (here the second input equal to 0) is not used. It is only used as a callback trigger.
        database_files, default_value = refresh_database_list(
            "/path/that/does/not/exist", None
        )
        self.assertEqual(database_files, [])
        self.assertEqual(default_value, "")

        # Assumes that in the current testing directory there are no .db3 files! Pretty safe assumption I hope -_-,
        database_files, default_value = refresh_database_list("./", None)
        self.assertEqual(database_files, [])
        self.assertEqual(default_value, "")

        db_dir = Path(self.db_path).parent
        database_files, default_value = refresh_database_list(db_dir, None)
        # We only check the names of the file so we do not have to deal with different absolute paths here locally and
        # in CI
        self.assertEqual(
            database_files[0]["label"],
            "dataset-calib-imu4_512_16.db3",
            "Are you sure there is no other database file in {dir} that 'sorts' before the tested one?".format(
                dir=db_dir
            ),
        )
        self.assertEqual(Path(default_value).name, "dataset-calib-imu4_512_16.db3")

    def test_load_database_to_store(self):
        raw_camera_data, raw_imu_data, metadata = load_database_to_store(None)
        self.assertIsNone(raw_camera_data)
        self.assertIsNone(raw_imu_data)
        self.assertIsNone(metadata)

        raw_camera_data, raw_imu_data, metadata = load_database_to_store(
            "/does/not/exist.db3"
        )
        self.assertIsNone(raw_camera_data)
        self.assertIsNone(raw_imu_data)
        self.assertIsNone(metadata)

        raw_camera_data, raw_imu_data, metadata = load_database_to_store(
            "/not/a/database/file.txt"
        )
        self.assertIsNone(raw_camera_data)
        self.assertIsNone(raw_imu_data)
        self.assertIsNone(metadata)

        raw_camera_data, raw_imu_data, metadata = load_database_to_store(self.db_path)
        self.assertIn("/cam0/image_raw", raw_camera_data)
        self.assertIn("/cam1/image_raw", raw_camera_data)
        self.assertIn("/imu0", raw_imu_data)

        statistics, timestamps = metadata
        # Check statistics
        self.assertIn(SensorType.Camera, statistics)
        self.assertIn(SensorType.Imu, statistics)
        self.assertIn("/cam0/image_raw", statistics[SensorType.Camera])
        self.assertIn("/imu0", statistics[SensorType.Imu])
        # Check timestamps
        self.assertIn(SensorType.Camera, timestamps)
        self.assertIn(SensorType.Imu, timestamps)
        self.assertIn("/cam0/image_raw", timestamps[SensorType.Camera])
        self.assertIn("/imu0", timestamps[SensorType.Imu])

    def test_get_sensor_names(self):
        sensor_names, default_value = get_sensor_names({}, SensorType.Camera)
        self.assertEqual(sensor_names, [])
        self.assertEqual(default_value, "")

        statistics = {SensorType.Camera: {"/sensor_1": {}, "/sensor_2": {}}}
        sensor_names, default_value = get_sensor_names(statistics, SensorType.Camera)
        self.assertEqual(sensor_names, ["/sensor_1", "/sensor_2"])
        self.assertEqual(default_value, "/sensor_1")
