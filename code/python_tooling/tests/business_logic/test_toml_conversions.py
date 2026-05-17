import unittest

from business_logic.toml_conversions import toml_to_intrinsic_array
from database.types import CameraModel


class TestTomlConversions(unittest.TestCase):
    def test_toml_to_intrinsic_array(self):
        toml_str = """\
            alpha = 0.5
            cx = 256.0
            cy = 256.0
            f = 160.0
            xi = 0.0
            """
        result = toml_to_intrinsic_array(toml_str, CameraModel.DoubleSphere)
        self.assertEqual(result, [160.0, 256.0, 256.0, 0.0, 0.5])

        toml_str = """\
            cx = 256.0
            cy = 256.0
            f = 160.0
            """
        result = toml_to_intrinsic_array(toml_str, CameraModel.Pinhole)
        self.assertEqual(result, [160.0, 256.0, 256.0])

        toml_str = """\
            cx = 256.0
            cy = 256.0
            f = 160.0
            k1 = 0.1
            k2 = 0.2
            p1 = 0.3
            p2 = 0.4
            """
        result = toml_to_intrinsic_array(toml_str, CameraModel.PinholeRadtan4)
        self.assertEqual(result, [160.0, 256.0, 256.0, 0.1, 0.2, 0.3, 0.4])

        toml_str = """\
            cx = 256.0
            cy = 256.0
            f = 160.0
            xi = 0.0
            """
        result = toml_to_intrinsic_array(toml_str, CameraModel.UnifiedCameraModel)
        self.assertEqual(result, [160.0, 256.0, 256.0, 0.0])