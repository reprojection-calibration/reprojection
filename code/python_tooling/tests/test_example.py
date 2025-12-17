import unittest

from database.example import add_one
from database.example import load_extracted_targets


class TestStringMethods(unittest.TestCase):

    def test_add_one(self):
        self.assertEqual(add_one(3), 4)

    def test_isupper(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())

    def test_split(self):
        s = 'hello world'
        self.assertEqual(s.split(), ['hello', 'world'])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)


class TestDatabaseConnections(unittest.TestCase):
    def test_extracted_target_loading(self):
        # WARN(Jack): Is there a better way to define this global value here? We might want to test inside of the
        # container and sometimes outside.
        db_path = "/home/stable-genius-gram/github/reprojection-calibration/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"
        load_extracted_targets(db_path)


if __name__ == '__main__':
    unittest.main()
