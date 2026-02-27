import os
import unittest

from database.measurement_data_formatting import (
    process_extracted_targets_table,
    process_images_table,
    process_imu_data_table,
)
from database.sql_table_loading import (
    load_extracted_targets_table,
    load_images_table,
    load_imu_data_table,
    load_poses_table,
    load_reprojection_errors_table,
)


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

    def test_process_imu_data_table(self):
        data = process_imu_data_table(None)
        self.assertIsNone(data)

        table = load_imu_data_table(self.db_path)
        data = process_imu_data_table(table)

        self.assertEqual(len(data), 1)
        self.assertEqual(list(data.keys()), ["/imu0"])

        self.assertTrue("measurements" in data["/imu0"])
        self.assertEqual(len(data["/imu0"]["measurements"]), 8770)
