import unittest
import os

from database.load_extracted_targets import load_extracted_targets_df
from database.load_poses import load_poses_df
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
        image_n = cam0[0]
        self.assertEqual(image_n['timestamp_ns'], 1520528314264184064)
        self.assertIsNone(image_n['data']) # The checked in database has no images

    def test_pose_loading(self):
        # If we make a query to a non-existent table that returns no data we just get an object with length zero
        df = load_poses_df(self.db_path, "does_not_exist", "also_not_there")
        self.assertIsNone(df)

        # At time of writing there is no camera pose data in the test_data database
        df = load_poses_df(self.db_path, "camera", "initial")
        self.assertIsNone(df)

        # Load the external poses table and test some simple values
        df = load_poses_df(self.db_path, "external", "ground_truth")
        self.assertEqual(df.shape, (4730, 9))

        # At this time there are only mocap poses so the shape is the same!
        mocap_poses = df.loc[df['sensor_name'] == '/mocap0']
        self.assertEqual(mocap_poses.shape, (4730, 9))

        pose_i = df.loc[df["timestamp_ns"] == 1520528318209068667]
        self.assertEqual(pose_i.shape, (1, 9))

    def test_extracted_target_loading(self):
        # Query nonexistent table
        df = load_extracted_targets_df('nonexistent.db3')
        self.assertIsNone(df)

        df = load_extracted_targets_df(self.db_path)

        # Two cameras
        self.assertEqual(df.shape, (1758, 3))
        # Each with 879 extracted targets
        cam0 = df.loc[df['sensor_name'] == '/cam0/image_raw']
        self.assertEqual(cam0.shape, (879, 3))
        cam1 = df.loc[df['sensor_name'] == '/cam1/image_raw']
        self.assertEqual(cam1.shape, (879, 3))

        # Check the dimensions of one of the loaded extracted targets
        cam0_target_i = cam0.loc[cam0['timestamp_ns'] == 1520528314264184064, 'data'].iloc[0]
        self.assertEqual(len(cam0_target_i['pixels']), 144)
        self.assertEqual(len(cam0_target_i['points']), 144)
        self.assertEqual(len(cam0_target_i['indices']), 144)


if __name__ == '__main__':
    unittest.main()
