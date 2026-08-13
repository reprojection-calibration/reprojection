import unittest

from database.calculate_metadata import count_data, reference_timestamps
from database.types import SensorType


class TestCalculateMetadata(unittest.TestCase):

    def test_count_data(self):
        raw_data = {
            "/cam0/image_raw": {
                "type": SensorType.Camera,
                "measurements": {"images": {0: None}},
            }
        }
        metadata = count_data(raw_data)

        gt_metadata = {
            "/cam0/image_raw": {
                "type": SensorType.Camera,
                "measurements": {"images": 1},
            }
        }

        self.assertEqual(metadata, gt_metadata)

    def test_reference_timestamps(self):
        timestamps = reference_timestamps(None)
        self.assertIsNone(timestamps)

        raw_data = {
            "/cam0/image_raw": {
                "type": SensorType.Camera,
                "measurements": {"images": {0: None, 1: None}},
            }
        }
        timestamps = reference_timestamps(raw_data)

        # Check arbitrarily that its sorted and the first and last timestamp of one of the entries
        cam0_image_timestamps = timestamps["/cam0/image_raw"]["measurements"]["images"]
        self.assertEqual(cam0_image_timestamps, sorted(cam0_image_timestamps))
        self.assertEqual(cam0_image_timestamps[0], "0")
        self.assertEqual(cam0_image_timestamps[-1], "1")
