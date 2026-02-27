import os
import sqlite3
import tempfile
import unittest
import pandas as pd

from database.data_formatting import (
    load_data,
    process_extracted_targets_table,
    process_images_table,
    process_imu_data_table,
    process_poses_table,
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


# TODO(Jack): Copy and pasted, put into common package!
def execute_sql(sql_statement, db_path):
    conn = sqlite3.connect(db_path)

    cursor = conn.cursor()
    cursor.execute(sql_statement)
    conn.commit()

    conn.close()


class TestDatabaseMeasurementDataFormatting(unittest.TestCase):
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

    def test_process_poses_table(self):
        data = process_poses_table(None, {})
        self.assertIsNone(data)

        pose_data = {"step_name": ["linear_pose_initialization", "linear_pose_initialization"],
                     "sensor_name": ["/cam0/image_raw", "/cam0/image_raw"],
                     "timestamp_ns": [0, 1], "rx": [0, 0], "ry": [0, 0], "rz": [0, 0], "x": [0, 0], "y": [0, 0],
                     "z": [0, 0]}
        table = pd.DataFrame(pose_data)

        self.assertRaises(KeyError, process_poses_table, table, {})

        images_table = load_images_table(self.db_path)
        data = process_images_table(images_table)

        process_poses_table(table, data)

        self.assertTrue("poses" in data["/cam0/image_raw"])
        self.assertTrue("linear_pose_initialization" in data["/cam0/image_raw"]["poses"])
        self.assertEqual(len(data["/cam0/image_raw"]["poses"]["linear_pose_initialization"]), 2)

    def test_process_imu_data_table(self):
        data = process_imu_data_table(None)
        self.assertIsNone(data)

        table = load_imu_data_table(self.db_path)
        data = process_imu_data_table(table)

        self.assertEqual(len(data), 1)
        self.assertEqual(list(data.keys()), ["/imu0"])

        self.assertTrue("measurements" in data["/imu0"])
        self.assertEqual(len(data["/imu0"]["measurements"]), 8770)

    def test_load_data(self):
        data = load_data("nonexistent.db3")
        self.assertIsNone(data)

        data = load_data(self.db_path)

        self.assertEqual(len(data), 2)
        self.assertEqual(list(data.keys()), [SensorType.Camera, SensorType.Imu])

        self.assertIsNotNone(data[SensorType.Camera])
        self.assertIsNotNone(data[SensorType.Imu])
