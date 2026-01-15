import os
import unittest
from pathlib import Path

from dashboard.callbacks.data_loading import (
    load_database_to_store,
    refresh_database_list,
    refresh_sensor_list,
)


class TestDatabaseCameraCalibrationData(unittest.TestCase):
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

        # Assumes that in the current testing directory there are no .db3 files! Pretty safe assumption I hope -_-
        list, default_value = refresh_database_list("./", 0)
        self.assertEqual(list, [])
        self.assertEqual(default_value, "")

        db_dir = Path(self.db_path).parent
        list, default_value = refresh_database_list(db_dir, 0)
        # We only check the name of the file so we can ignore the absolute paths locally vs. in CI or if people add other
        # databases locally for debugging.
        self.assertEqual(list[0]["label"], "dataset-calib-imu4_512_16.db3")
        self.assertEqual(Path(default_value).name, "dataset-calib-imu4_512_16.db3")
