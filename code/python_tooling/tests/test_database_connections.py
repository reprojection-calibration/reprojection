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

        data = split_images_by_sensor(df)

        cam0 = data['/cam0/image_raw']
        self.assertEqual(len(cam0), 879)

        # Check the dimensions of one of the loaded extracted targets
        image_i = cam0[0]
        self.assertEqual(image_i['timestamp_ns'], 1520528314264184064)
        self.assertIsNone(image_i['data'])  # The checked in database has no images

    def test_pose_loading(self):
        # If we make a query to a non-existent table that returns no data we just get an object with length zero
        df = load_poses_df(self.db_path, "does_not_exist", "also_not_there")
        self.assertIsNone(df)

        df_initial, df_optimized, df_external = load_calibration_poses(self.db_path)
        self.assertEqual(df_initial.shape, (0, 9))
        self.assertEqual(df_optimized.shape, (0, 9))
        self.assertEqual(df_external.shape, (4730, 9))

    def test_extracted_target_loading(self):
        # Query nonexistent table
        df = load_extracted_targets_df('nonexistent.db3')
        self.assertIsNone(df)

        df = load_extracted_targets_df(self.db_path)
        self.assertEqual(df.shape, (1758, 3))

        data = split_extracted_targets_by_sensor(df)

        cam0 = data['/cam0/image_raw']
        self.assertEqual(len(cam0), 879)
        cam1 = data['/cam1/image_raw']
        self.assertEqual(len(cam1), 879)

        # Check the dimensions of one of the loaded extracted targets
        target_i = cam0[0]
        self.assertEqual(len(target_i['pixels']), 144)
        self.assertEqual(len(target_i['points']), 144)
        self.assertEqual(len(target_i['indices']), 144)


if __name__ == '__main__':
    unittest.main()
