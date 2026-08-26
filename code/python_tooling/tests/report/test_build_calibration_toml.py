import unittest
from textwrap import dedent

import pandas as pd

from database.data_formatting import Workflow
from report.build_calibration_toml import (
    build_extrinsic_toml,
    build_intrinsic_toml,
)


class TestBuildCameraTomls(unittest.TestCase):
    def test_build_intrinsic_toml(self):
        workflow = Workflow(
            id=1,
            type="cam",
            asset_group_signature="cam0",
            assets={
                "camera": {
                    "id": 1,
                    "index": 0,
                    "name": "/cam0/image_raw",
                },
            },
            steps={
                "camera_info": 1,
                "bundle_adjustment": 2,
            },
        )

        camera_info = pd.Series(
            {
                "step_id": 1,
                "asset_id": 1,
                "camera_model": "pinhole_radtan4",
                "height": 720,
                "width": 1080,
            }
        )

        camera_intrinsics = pd.DataFrame(
            [
                {
                    "step_id": 2,
                    "asset_id": 1,
                    "camera_model": "pinhole_radtan4",
                    "data": """
                            cx = 256.0
                            cy = 256.0
                            f = 160.0
                            k1 = 0.1
                            k2 = 0.2
                            p1 = 0.3
                            p2 = 0.4
                        """,
                },
            ]
        ).set_index(["step_id", "asset_id"])

        raw_data = {
            "camera_info": camera_info,
            "intrinsics": camera_intrinsics,
        }

        result = build_intrinsic_toml(workflow, raw_data)

        result_gt = """\
        [workflow1.cam0]
        sensor_id = '/cam0/image_raw'
        camera_model = 'pinhole_radtan4'
        intrinsics = [160.0, 256.0, 256.0, 0.1, 0.2, 0.3, 0.4]
        resolution = [720, 1080]
        """

        self.assertEqual(result, dedent(result_gt))

    def test_build_extrinsic_toml(self):
        workflow = Workflow(
            id=3,
            type="cam_imu",
            asset_group_signature="cam0_imu0",
            assets={
                "camera": {
                    "id": 1,
                    "index": 0,
                    "name": "frame_a_1",
                },
                "imu": {
                    "id": 2,
                    "index": 0,
                    "name": "frame_b_1",
                },
            },
            steps={
                "extrinsic_optimization": 5,
            },
        )

        extrinsics = pd.DataFrame(
            [
                {
                    "step_id": 5,
                    "asset_a_id": 1,
                    "asset_b_id": 2,
                    "rx": 1,
                    "ry": 2,
                    "rz": 3,
                    "x": 4,
                    "y": 5,
                    "z": 6,
                },
            ]
        )

        raw_data = {
            "extrinsics": extrinsics,
        }

        result = build_extrinsic_toml(workflow, raw_data)

        result_gt = """\
        [workflow3.extrinsic0]
        frame_a = 'frame_a_1'
        frame_b = 'frame_b_1'
        tf_a_b = [
          [-0.694920557641, 0.713520990528, 0.0892928588619, 4],
          [-0.192006972792, -0.303785044339, 0.933192353824, 5],
          [0.692978167742, 0.631349699384, 0.34810747783, 6],
          [0, 0, 0, 1]
        ]
        """

        self.assertEqual(result, dedent(result_gt))
