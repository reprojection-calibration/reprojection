import os

import unittest

from database.data_formatting import (
    load_data,
)

from database.calculate_metadata import count_structure
from database.types import SensorType


class TestDatabaseCalculateMetadata(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_count_structure(self):
        metadata = count_structure(None)
        self.assertIsNone(metadata)

        data = load_data(self.db_path)
        metadata = count_structure(data)

        gt_metadata = {
            SensorType.Camera: {
                "/cam0/image_raw": {"measurements": {"images": 879, "targets": 879}},
                "/cam1/image_raw": {"measurements": {"images": 879, "targets": 879}},
            },
            SensorType.Imu: {"/imu0": {"measurements": 8770}},
        }

        self.assertEqual(metadata, gt_metadata)
