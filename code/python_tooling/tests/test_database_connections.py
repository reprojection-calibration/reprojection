import unittest
import os

from database.load_extracted_targets import load_all_extracted_targets
from database.load_poses import load_poses
from database.load_image_frame_data import load_image_frame_data


class TestDatabaseConnections(unittest.TestCase):
    # NOTE(Jack): This is a preemptive attempt to make it possible to execute the test locally or in the docker. Let's
    # see if this works or we just end up removing it later :)
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3")

    def test_load_image_frame_data(self):
        loaded_data = load_image_frame_data(self.db_path)
        self.assertEqual(len(loaded_data), 2)
        self.assertEqual(len(loaded_data["/cam0/image_raw"]['data']), 879)
        self.assertEqual(len(loaded_data["/cam1/image_raw"]['data']), 879)

        image_data_i = loaded_data["/cam0/image_raw"]['data'][1520528314264184064]
        self.assertEqual(len(image_data_i['external_pose']), 7)
        self.assertTrue('initial_pose' not in image_data_i)
        self.assertTrue('optimized_pose' not in image_data_i)

        image_data_i = loaded_data["/cam1/image_raw"]['data'][1520528314264184064]
        self.assertEqual(len(image_data_i['external_pose']), 7)

    def test_pose_loading(self):
        # If we make a query to a non-existent table that returns no data we just get an object with length zero
        df = load_poses(
            self.db_path, "does_not_exist", "also_not_there")
        self.assertIsNone(df)

        # At time of writing there is no camera pose data in the test_data database
        df = load_poses(self.db_path, "camera", "initial")
        self.assertEqual(df.size, 0)

        # Load the external poses table and test some simple values
        df = load_poses(self.db_path, "external", "ground_truth")
        self.assertEqual(df.shape, (4730, 9))

        # At this time there are only mocap poses so the shape is the same!
        mocap_poses = df.loc[df['sensor_name'] == '/mocap0']
        self.assertEqual(mocap_poses.shape, (4730, 9))

        pose_i = df.loc[df["timestamp_ns"] == 1520528318209068667]
        self.assertEqual(pose_i.shape, (1, 9))

    def test_extracted_target_loading(self):
        loaded_data = load_all_extracted_targets(self.db_path)

        # Two cameras
        self.assertEqual(len(loaded_data), 2)
        # Each with 879 extracted targets
        self.assertEqual(len(loaded_data["/cam0/image_raw"]['data']), 879)
        self.assertEqual(len(loaded_data["/cam1/image_raw"]['data']), 879)

        # Check that the dimensions of one of the loaded extracted targets matches our expectations (ex. 2,3,2)
        extracted_target_i = loaded_data["/cam0/image_raw"]['data'][1520528314264184064]
        self.assertEqual(len(extracted_target_i["extracted_target"]['pixels']), 144)
        self.assertEqual(len(extracted_target_i["extracted_target"]['points']), 144)
        self.assertEqual(len(extracted_target_i["extracted_target"]['indices']), 144)


if __name__ == '__main__':
    unittest.main()
