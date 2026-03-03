import unittest

import numpy as np

from database.geometry import InvertSe3


class TestGeometry(unittest.TestCase):
    def test_invert_se3(self):
        se3 = np.array([0, 0, 0, 0, 0, 0])
        se3_inv = InvertSe3(se3)
        self.assertEqual(se3_inv, [0, 0, 0, 0, 0, 0])

        se3 = np.array([0, 0, 0, 1, 2, 3])
        se3_inv = InvertSe3(se3)
        self.assertEqual(se3_inv, [0, 0, 0, -1, -2, -3])

        se3 = np.array([np.pi / 2, 0, 0, 0, 0, 0])
        se3_inv = InvertSe3(se3)
        np.testing.assert_almost_equal(se3_inv, [-np.pi / 2.0, 0, 0, 0, 0, 0])

        se3 = np.array([np.pi / 2, 0, 0, 1, 2, 3])
        se3_inv = InvertSe3(se3)
        np.testing.assert_almost_equal(se3_inv, [-np.pi / 2.0, 0, 0, -1, -3, 2])
