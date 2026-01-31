import os
import unittest
from pathlib import Path

from dashboard.callbacks.data_loading import (
    load_database_to_store,
    refresh_database_list,
    refresh_sensor_list,
)


class TestDashboardCallbacksDataLoading(unittest.TestCase):
    # NOTE(Jack): See note in TestDatabaseCameraCalibrationData
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_refresh_database_list(self):
        # The value of n_clicks (here the second input equal to 0) is not used. It is only used as a callback trigger.
        list, default_value = refresh_database_list("/does/not/exist", 0)
        self.assertEqual(list, [])
        self.assertEqual(default_value, "")

        # Assumes that in the current testing directory there are no .db3 files! Pretty safe assumption I hope -_-.
        list, default_value = refresh_database_list("./", 0)
        self.assertEqual(list, [])
        self.assertEqual(default_value, "")

        db_dir = Path(self.db_path).parent
        list, default_value = refresh_database_list(db_dir, 0)
        # We only check the name of the file so we can ignore the absolute paths locally vs. in CI or if people add other
        # databases locally for debugging.
        self.assertEqual(
            list[0]["label"],
            "dataset-calib-imu4_512_16.db3",
            "Are you sure there is no other database file in {dir} that 'sorts' before the tested one?".format(
                dir=db_dir
            ),
        )
        self.assertEqual(Path(default_value).name, "dataset-calib-imu4_512_16.db3")

    def test_load_database_to_store(self):
        raw_data, processed_data = load_database_to_store(None)
        self.assertIsNone(raw_data)
        self.assertIsNone(processed_data)

        raw_data, processed_data = load_database_to_store("/does/not/exist.db3")
        self.assertIsNone(raw_data)
        self.assertIsNone(processed_data)

        raw_data, processed_data = load_database_to_store("/not/a/database/file.txt")
        self.assertIsNone(raw_data)
        self.assertIsNone(processed_data)

        raw_camera_data, raw_imu_data, processed_data = load_database_to_store(
            self.db_path
        )
        self.assertIn("/cam0/image_raw", raw_camera_data)
        self.assertIn("/imu0", raw_imu_data)

        # TODO(Jack): Better more meaningful name than indexable_timestamps!!!
        statistics, indexable_timestamps = processed_data
        self.assertIn("/cam0/image_raw", statistics)
        self.assertIn("/cam0/image_raw", indexable_timestamps)

    def test_refresh_sensor_list(self):
        list, default_value = refresh_sensor_list(None)
        self.assertEqual(list, [])
        self.assertEqual(default_value, "")

        list, default_value = refresh_sensor_list([{}, None])
        self.assertEqual(list, [])
        self.assertEqual(default_value, "")

        statistics = {"/sensor_1": {}, "/sensor_2": {}}
        list, default_value = refresh_sensor_list([statistics, None])
        self.assertEqual(list, ["/sensor_1", "/sensor_2"])
        self.assertEqual(default_value, "/sensor_1")
