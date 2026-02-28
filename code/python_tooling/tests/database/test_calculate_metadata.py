import os
import unittest

from database.calculate_metadata import count_data, reference_timestamps
from database.data_formatting import load_data
from database.types import SensorType


class TestCalculateMetadata(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.db3"
        )

    def test_count_data(self):
        metadata = count_data(None)
        self.assertIsNone(metadata)

        data = load_data(self.db_path)
        metadata = count_data(data)

        gt_metadata = {
            "/cam0/image_raw": {
                "type": SensorType.Camera,
                "measurements": {"images": 879, "targets": 879},
            },
            "/cam1/image_raw": {
                "type": SensorType.Camera,
                "measurements": {"images": 879, "targets": 879},
            },
            "/imu0": {"type": SensorType.Imu, "measurements": 8770},
        }

        self.assertEqual(metadata, gt_metadata)

    def test_reference_timestamps(self):
        timestamps = reference_timestamps(None)
        self.assertIsNone(timestamps)

        data = load_data(self.db_path)
        timestamps = reference_timestamps(data)

        # Check arbitrarily that its sorted and the first and last timestamp of one of the entries
        # TODO(Jack): Why does black format this so weird???
        cam0_image_timestamps = timestamps["/cam0/image_raw"]["measurements"]["images"]
        self.assertEqual(cam0_image_timestamps, sorted(cam0_image_timestamps))
        self.assertEqual(cam0_image_timestamps[0], "1520528314264184064")
        self.assertEqual(cam0_image_timestamps[-1], "1520528358165555968")
