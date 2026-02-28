import os
import unittest

import pandas as pd

from database.data_formatting import (
    load_data,
    process_extracted_targets_table,
    process_images_table,
    process_imu_data_table,
    process_poses_table,
    process_reprojection_error_table,
)
from database.sql_statement_loading import load_sql
from database.sql_table_loading import (
    load_extracted_targets_table,
    load_images_table,
    load_imu_data_table,
    load_poses_table,
    load_reprojection_errors_table,
)
from database.types import SensorType


class TestDataFormatting(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_process_images_table(self):
        data = process_images_table(None)
        self.assertIsNone(data)

        table = load_images_table(self.db_path)
        data = process_images_table(table)

        self.assertEqual(len(data), 2)
        self.assertEqual(list(data.keys()), ["/cam0/image_raw", "/cam1/image_raw"])

        def check(data):
            self.assertTrue("type" in data)
            self.assertTrue(SensorType.Camera in data["type"])

            self.assertTrue("measurements" in data)
            self.assertTrue("images" in data["measurements"])
            self.assertEqual(len(data["measurements"]["images"]), 879)

        check(data["/cam0/image_raw"])
        check(data["/cam1/image_raw"])

    def test_process_extracted_targets_table(self):
        data = process_extracted_targets_table(None, None)
        self.assertIsNone(data)

        # Loading the targets into an empty dictionary throws an error because it depends on process_images_table having
        # partially filled the table already.
        table = load_extracted_targets_table(self.db_path)
        self.assertRaises(KeyError, process_extracted_targets_table, table, {})

        # Load the images table into the data matrix - now we can run process_extracted_targets_table to fill out the
        # data dictionary with the extracted targets.
        images_table = load_images_table(self.db_path)
        data = process_images_table(images_table)

        process_extracted_targets_table(table, data)

        def check(data):
            self.assertTrue("measurements" in data)
            self.assertTrue("targets" in data["measurements"])
            self.assertEqual(len(data["measurements"]["targets"]), 879)

        check(data["/cam0/image_raw"])
        check(data["/cam1/image_raw"])

    # TODO(Jack): The following two tests are good examples how our testing is hard to read! For both tests we need to
    #  create relatively complicated state setups to check the foreign key constraints, and we do it all right in the
    #  middle of the test. We need to find a way to streamline this for all tests!
    def test_process_poses_table(self):
        data = process_poses_table(None, {})
        self.assertIsNone(data)

        # Pose loading has no timestamp foreign key requirements so we can use any random timestamps here.
        pose_data = {
            "step_name": ["linear_pose_initialization", "linear_pose_initialization"],
            "sensor_name": ["/cam0/image_raw", "/cam0/image_raw"],
            "timestamp_ns": [0, 1],
            "rx": [0, 0],
            "ry": [0, 0],
            "rz": [0, 0],
            "x": [0, 0],
            "y": [0, 0],
            "z": [0, 0],
        }
        table = pd.DataFrame(pose_data)

        self.assertRaises(KeyError, process_poses_table, table, {})

        images_table = load_images_table(self.db_path)
        data = process_images_table(images_table)

        process_poses_table(table, data)

        self.assertTrue("poses" in data["/cam0/image_raw"])
        self.assertTrue(
            "linear_pose_initialization" in data["/cam0/image_raw"]["poses"]
        )
        self.assertEqual(
            len(data["/cam0/image_raw"]["poses"]["linear_pose_initialization"]), 2
        )

    def test_process_reprojection_error_table(self):
        data = process_reprojection_error_table(None, {})
        self.assertIsNone(data)

        reprojection_data = {
            "step_name": ["linear_pose_initialization", "linear_pose_initialization"],
            "sensor_name": ["/cam0/image_raw", "/cam0/image_raw"],
            "timestamp_ns": [1520528314264184064, 1520528314314184960],
            "data": [{}, {}],
        }
        table = pd.DataFrame(reprojection_data)

        self.assertRaises(KeyError, process_reprojection_error_table, table, {})

        images_table = load_images_table(self.db_path)
        data = process_images_table(images_table)

        pose_data = {
            "step_name": ["linear_pose_initialization", "linear_pose_initialization"],
            "sensor_name": ["/cam0/image_raw", "/cam0/image_raw"],
            "timestamp_ns": [1520528314264184064, 1520528314314184960],
            "rx": [0, 0],
            "ry": [0, 0],
            "rz": [0, 0],
            "x": [0, 0],
            "y": [0, 0],
            "z": [0, 0],
        }
        pose_table = pd.DataFrame(pose_data)
        process_poses_table(pose_table, data)

        process_reprojection_error_table(table, data)

        self.assertTrue("poses" in data["/cam0/image_raw"])
        self.assertTrue(
            "linear_pose_initialization"
            in data["/cam0/image_raw"]["reprojection_error"]
        )
        self.assertEqual(
            len(
                data["/cam0/image_raw"]["reprojection_error"][
                    "linear_pose_initialization"
                ]
            ),
            2,
        )

    def test_process_imu_data_table(self):
        data = process_imu_data_table(None)
        self.assertIsNone(data)

        table = load_imu_data_table(self.db_path)
        data = process_imu_data_table(table)

        self.assertEqual(len(data), 1)
        self.assertEqual(list(data.keys()), ["/imu0"])

        self.assertTrue("type" in data["/imu0"])
        self.assertTrue(SensorType.Imu in data["/imu0"]["type"])

        self.assertTrue("measurements" in data["/imu0"])
        self.assertEqual(len(data["/imu0"]["measurements"]), 8770)

    def test_load_data(self):
        data = load_data("nonexistent.db3")
        self.assertIsNone(data)

        data = load_data(self.db_path)

        self.assertEqual(len(data), 3)
        self.assertEqual(
            list(data.keys()), ["/cam0/image_raw", "/cam1/image_raw", "/imu0"]
        )
