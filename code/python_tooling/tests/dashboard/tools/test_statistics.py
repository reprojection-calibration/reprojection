import unittest

from dashboard.tools.statistics import extract_labeled_metadata
from database.calculate_metadata import count_data
from database.data_formatting import load_data
from database.types import SensorType


class TestStatistics(unittest.TestCase):
    def test_extract_labeled_metadata(self):
        # NOTE(Jack): We add the test1/test2/test3 case to make sure that for more deeply nested dicts it still acts
        # like we expect! As of now we normally do not see that given the current data structure.
        sensor_metadata = {
            "type": SensorType.Camera,
            "measurements": {"images": 879, "targets": 879},
            "test1": {"test2": {"test3": 0}},
        }

        gt_result = [
            (["type"], SensorType.Camera),
            (["measurements", "images"], 879),
            (["measurements", "targets"], 879),
            (["test1", "test2", "test3"], 0),
        ]

        result = extract_labeled_metadata(sensor_metadata)
        self.assertEqual(result, gt_result)
