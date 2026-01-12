import unittest
import os

from database.load_extracted_targets import load_extracted_targets_df, split_extracted_targets_by_sensor
from database.load_poses import load_poses_df, load_calibration_poses
from database.load_images import split_images_by_sensor, load_images_df


class TestDatabaseConnections(unittest.TestCase):
    # NOTE(Jack): This is a preemptive attempt to make it possible to execute the test locally or in the docker. Let's
    # see if this works or we just end up removing it later :)
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3")

    def test_load_images(self):
        df = load_images_df('nonexistent.db3')
        self.assertIsNone(df)

        df = load_images_df(self.db_path)
        self.assertEqual(df.shape, (1758, 3))

    def test_extracted_target_loading(self):
        # Query nonexistent table
        df = load_extracted_targets_df('nonexistent.db3')
        self.assertIsNone(df)

        df = load_extracted_targets_df(self.db_path)
        self.assertEqual(df.shape, (1758, 3))




if __name__ == '__main__':
    unittest.main()
