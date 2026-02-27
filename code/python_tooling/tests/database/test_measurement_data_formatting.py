import os
import unittest

from database.measurement_data_formatting import process_images_table
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
