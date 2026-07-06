import unittest
from textwrap import dedent

import pandas as pd

from report.build_calibration_toml import build_intrinsic_toml


class TestBuildCameraTomls(unittest.TestCase):
    def test_build_intrinsic_toml(self):
        camera_info_data = {
            "sensor_name": ["/cam0/image_raw", "/cam1/image_raw"],
            "camera_model": ["pinhole_radtan4", "double_sphere"],
            "height": [720, 720],
            "width": [1080, 1080],
        }
        camera_info_table = pd.DataFrame(camera_info_data)

        camera_intrinsics_data = {
            "step_name": [
                "bundle_adjustment",
                "bundle_adjustment",
                "intrinsic_initialization",
            ],
            "sensor_name": ["/cam0/image_raw", "/cam1/image_raw", "/cam0/image_raw"],
            "camera_model": ["pinhole_radtan4", "double_sphere", "pinhole_radtan4"],
            "intrinsics": [
                """\
            cx = 256.0
            cy = 256.0
            f = 160.0
            k1 = 0.1
            k2 = 0.2
            p1 = 0.3
            p2 = 0.4
            """,
                """\
            alpha = 0.5
            cx = 256.0
            cy = 256.0
            f = 160.0
            xi = 0.0
            """,
                """\
            cx = 256.0
            cy = 256.0
            f = 160.0
            k1 = 0.0
            k2 = 0.0
            p1 = 0.0
            p2 = 0.0
            """,
            ],
        }
        camera_intrinsics_table = pd.DataFrame(camera_intrinsics_data)

        result = build_intrinsic_toml(camera_info_table, camera_intrinsics_table)

        result_gt = """\
        [cam0]
        sensor_id = '/cam0/image_raw'
        # https://github.com/reprojection-calibration/reprojection#camera-models
        camera_model = 'pinhole_radtan4'
        intrinsics = [160.0, 256.0, 256.0, 0.1, 0.2, 0.3, 0.4]
        resolution = [720, 1080]
        
        [cam1]
        sensor_id = '/cam1/image_raw'
        # https://github.com/reprojection-calibration/reprojection#camera-models
        camera_model = 'double_sphere'
        intrinsics = [160.0, 256.0, 256.0, 0.0, 0.5]
        resolution = [720, 1080]
        """

        self.assertEqual(result, dedent(result_gt))
