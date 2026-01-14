import os
import unittest

import pandas as pd

from database.load_camera_calibration_data import (
    get_camera_calibration_data_statistics, load_camera_calibration_data)
from database.load_camera_poses import \
    add_camera_poses_df_to_camera_calibration_data
from database.load_extracted_targets import (
    add_extracted_targets_df_to_camera_calibration_data,
    load_extracted_targets_df)
from database.load_images import (image_df_to_camera_calibration_data,
                                  load_images_df)
from database.load_reprojection_errors import (
    add_reprojection_errors_df_to_camera_calibration_data,
    load_reprojection_errors_df)


class TestDatabaseCameraCalibrationData(unittest.TestCase):
    # NOTE(Jack): This is a preemptive attempt to make it possible to execute the test locally or in the docker. Let's
    # see if this works or we just end up removing it later :)
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_load_camera_calibration_data(self):
        data = load_camera_calibration_data(self.db_path)
        statistics = get_camera_calibration_data_statistics(data)

        self.assertEqual(len(statistics.keys()), 2)

        cam0_statistics = statistics["/cam0/image_raw"]
        self.assertEqual(len(cam0_statistics.keys()), 7)

        self.assertEqual(cam0_statistics["total_frames"], 879)
        self.assertEqual(cam0_statistics["frames_with_image"], 0)
        self.assertEqual(cam0_statistics["frames_with_extracted_target"], 879)
        self.assertEqual(cam0_statistics["frames_with_initial_pose"], 0)
        self.assertEqual(cam0_statistics["frames_with_initial_reprojection_error"], 0)
        self.assertEqual(cam0_statistics["frames_with_optimized_pose"], 0)
        self.assertEqual(cam0_statistics["frames_with_optimized_reprojection_error"], 0)

    def test_image_df_to_camera_calibration_data(self):
        df = load_images_df(self.db_path)
        data = image_df_to_camera_calibration_data(df)

        self.assertEqual(len(data.keys()), 2)
        # At this time it only contains the "frames" key and no other information/metadata.
        self.assertEqual(len(data["/cam0/image_raw"]), 1)

        self.assertEqual(len(data["/cam0/image_raw"]["frames"]), 879)
        self.assertEqual(len(data["/cam1/image_raw"]["frames"]), 879)

        # Check one random image to check that is it None - at this time the database has no actual images in it.
        self.assertIsNone(
            data["/cam0/image_raw"]["frames"][1520528314314184960]["image"]
        )

    def test_add_extracted_targets_df_to_camera_calibration_data(self):
        df = load_images_df(self.db_path)
        data = image_df_to_camera_calibration_data(df)

        df = load_extracted_targets_df(self.db_path)
        add_extracted_targets_df_to_camera_calibration_data(df, data)

        # Check that no new frames got added by accident.
        self.assertEqual(len(data["/cam0/image_raw"]["frames"]), 879)

        # Check one random target.
        extracted_target_i = data["/cam0/image_raw"]["frames"][1520528314314184960][
            "extracted_target"
        ]
        self.assertEqual(len(extracted_target_i.keys()), 3)
        self.assertEqual(len(extracted_target_i["pixels"]), 143)
        self.assertEqual(len(extracted_target_i["points"]), 143)
        self.assertEqual(len(extracted_target_i["indices"]), 143)

        # If we pass in an empty dictionary that has not already had the camera data loaded and initialized it will
        # throw an error. This is us essentially testing that we have implemented the database foreign key constraints
        # here in python. At least partially.
        self.assertRaises(
            RuntimeError, add_extracted_targets_df_to_camera_calibration_data, df, {}
        )

    def test_add_camera_poses_df_to_camera_calibration_data(self):
        df = load_images_df(self.db_path)
        data = image_df_to_camera_calibration_data(df)

        # We should use load_camera_poses_df() but as of time of writing there is no pose data in the test database.
        # Therefore, we manually create a pose dataframe. If at some later date pose data gets added to the database we
        # should refactor this to use that instead of hardcoding it here.
        pose_data = [
            {
                "timestamp_ns": 1520528314314184960,
                "sensor_name": "/cam0/image_raw",
                "type": "initial",
                "rx": 1,
                "ry": 2,
                "rz": 3,
                "x": -1,
                "y": -2,
                "z": -3,
            },
            {
                "timestamp_ns": 1520528314314184960,
                "sensor_name": "/cam0/image_raw",
                "type": "optimized",
                "rx": 11,
                "ry": 22,
                "rz": 33,
                "x": -11,
                "y": -22,
                "z": -33,
            },
        ]

        df = pd.DataFrame(pose_data)
        add_camera_poses_df_to_camera_calibration_data(df, data)

        # Check that no new frames got added by accident.
        self.assertEqual(len(data["/cam0/image_raw"]["frames"]), 879)

        # Check the artificially created poses.
        poses_i = data["/cam0/image_raw"]["frames"][1520528314314184960]["poses"]
        self.assertEqual(len(poses_i.keys()), 2)
        self.assertEqual(poses_i["initial"], [1, 2, 3, -1, -2, -3])
        self.assertEqual(poses_i["optimized"], [11, 22, 33, -11, -22, -33])

        # Foreign key constraint check, see comment in test_add_extracted_targets_df_to_camera_calibration_data.
        self.assertRaises(
            RuntimeError, add_camera_poses_df_to_camera_calibration_data, df, {}
        )

    def test_add_reprojection_errors_df_to_camera_calibration_data(self):
        df = load_images_df(self.db_path)
        data = image_df_to_camera_calibration_data(df)

        # See comment above test_add_camera_poses_df_to_camera_calibration_data about why we hardcode the data in here.
        reprojection_error_data = [
            {
                "timestamp_ns": 1520528314314184960,
                "sensor_name": "/cam0/image_raw",
                "type": "initial",
                "data": [[0, 1], [2, 3], [4, 5]],
            },
            {
                "timestamp_ns": 1520528314314184960,
                "sensor_name": "/cam0/image_raw",
                "type": "optimized",
                "data": [[0, 11], [22, 33], [44, 55]],
            },
        ]

        df = pd.DataFrame(reprojection_error_data)
        add_reprojection_errors_df_to_camera_calibration_data(df, data)

        # Check that no new frames got added by accident.
        self.assertEqual(len(data["/cam0/image_raw"]["frames"]), 879)

        # Check the artificially created poses.
        reprojection_errors_i = data["/cam0/image_raw"]["frames"][1520528314314184960][
            "reprojection_errors"
        ]
        self.assertEqual(len(reprojection_errors_i.keys()), 2)
        self.assertEqual(reprojection_errors_i["initial"], [[0, 1], [2, 3], [4, 5]])
        self.assertEqual(
            reprojection_errors_i["optimized"], [[0, 11], [22, 33], [44, 55]]
        )

        # Foreign key constraint check, see comment in test_add_extracted_targets_df_to_camera_calibration_data.
        self.assertRaises(
            RuntimeError, add_reprojection_errors_df_to_camera_calibration_data, df, {}
        )


if __name__ == "__main__":
    unittest.main()
