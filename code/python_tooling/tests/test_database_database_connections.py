import unittest
import os

from database.load_extracted_targets import load_extracted_targets_df
from database.load_camera_poses import load_camera_poses_df
from database.load_images import load_images_df
from database.load_reprojection_errors import load_reprojection_errors_df


class TestDatabaseDatabaseConnections(unittest.TestCase):
    # NOTE(Jack): This is a preemptive attempt to make it possible to execute the test locally or in the docker. Let's
    # see if this works or we just end up removing it later :)
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_load_images_df(self):
        df = load_images_df("nonexistent.db3")
        self.assertIsNone(df)

        df = load_images_df(self.db_path)
        self.assertEqual(df.shape, (1758, 3))

    def test_load_extracted_targets_df(self):
        df = load_extracted_targets_df("nonexistent.db3")
        self.assertIsNone(df)

        df = load_extracted_targets_df(self.db_path)
        self.assertEqual(df.shape, (1758, 3))

    def test_load_camera_poses_df(self):
        df = load_camera_poses_df("nonexistent.db3")
        self.assertIsNone(df)

        df = load_camera_poses_df(self.db_path)
        self.assertEqual(df.shape, (0, 9))

    def test_load_reprojection_errors_df(self):
        df = load_reprojection_errors_df("nonexistent.db3")
        self.assertIsNone(df)

        df = load_reprojection_errors_df(self.db_path)
        self.assertEqual(df.shape, (0, 4))


if __name__ == "__main__":
    unittest.main()
