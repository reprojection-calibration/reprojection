import unittest

from database.load_extracted_targets import load_extracted_targets


class TestDatabaseConnections(unittest.TestCase):
    def test_extracted_target_loading(self):
        # WARN(Jack): Is there a better way to define this global value here? We might want to test inside of the
        # container and sometimes outside.
        db_path = "/home/stable-genius-gram/github/reprojection-calibration/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"
        loaded_data = load_extracted_targets(db_path)

        # Two cameras
        self.assertEqual(len(loaded_data), 2)
        # Each with 879 extracted targets
        self.assertEqual(len(loaded_data["/cam0/image_raw"]), 879)
        self.assertEqual(len(loaded_data["/cam1/image_raw"]), 879)

        # Check that the dimensions of one of the loaded extracted targets matches our expectations (ex. 2,3,2)
        extracted_target_i = loaded_data["/cam0/image_raw"][1520528314264184064]
        self.assertEqual(extracted_target_i.pixels.shape, (144, 2))
        self.assertEqual(extracted_target_i.points.shape, (144, 3))
        self.assertEqual(extracted_target_i.indices.shape, (144, 2))


if __name__ == '__main__':
    unittest.main()
