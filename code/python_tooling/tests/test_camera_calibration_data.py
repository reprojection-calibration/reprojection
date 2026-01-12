import unittest
import os

from database.load_extracted_targets import load_extracted_targets_df, \
    add_extracted_targets_df_to_camera_calibration_data
from database.load_camera_poses import load_camera_poses_df
from database.load_images import image_df_to_camera_calibration_data, load_images_df


class TestCameraCalibrationData(unittest.TestCase):
    # NOTE(Jack): This is a preemptive attempt to make it possible to execute the test locally or in the docker. Let's
    # see if this works or we just end up removing it later :)
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3")

    def test_image_df_to_camera_calibration_data(self):
        df = load_images_df(self.db_path)
        data = image_df_to_camera_calibration_data(df)

        self.assertEqual(len(data.keys()), 2)
        # At this time it only contains the "frames" key and no other information/metadata
        self.assertEqual(len(data['/cam0/image_raw']), 1)

        self.assertEqual(len(data['/cam0/image_raw']['frames']), 879)
        self.assertEqual(len(data['/cam1/image_raw']['frames']), 879)

        # Check one random image to check that is it None - at this time the database has no actual images in it
        self.assertIsNone(data['/cam0/image_raw']['frames'][1520528314314184960]['image'])

    def test_extracted_targets_df_to_camera_calibration_data(self):
        df = load_images_df(self.db_path)
        data = image_df_to_camera_calibration_data(df)

        df = load_extracted_targets_df(self.db_path)
        add_extracted_targets_df_to_camera_calibration_data(df, data)

        # Check that no new frames got added by accident
        self.assertEqual(len(data['/cam0/image_raw']['frames']), 879)

        # Check one random target
        extracted_target_i = data['/cam0/image_raw']['frames'][1520528314314184960]['extracted_target']
        self.assertEqual(len(extracted_target_i.keys()), 3)
        self.assertEqual(len(extracted_target_i['pixels']), 143)
        self.assertEqual(len(extracted_target_i['points']), 143)
        self.assertEqual(len(extracted_target_i['indices']), 143)

        # If we pass in an empty dictionary that has not already had the camera data loaded and initialized it will
        # throw an error. This is us essentially testing that we have implemented the database foreign key constraints
        # here in python. At least partially.
        self.assertRaises(RuntimeError, add_extracted_targets_df_to_camera_calibration_data, df, {})


if __name__ == '__main__':
    unittest.main()
