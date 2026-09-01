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
                1: {
                    "id": 1,
                    "type": "camera",
                    "index": 0,
                    "name": "/cam0/image_raw",
                },
            },
            steps={
                1: {"type": "camera_info", "asset_group_signature": "1|"},
                2: {"type": "bundle_adjustment", "asset_group_signature": "1|"},
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

    def test_build_intrinsic_toml_exports_every_camera(self):
        workflow = Workflow(
            id=1,
            type="cam_imu",
            asset_group_signature="1|2|3|",
            assets={
                1: {"id": 1, "type": "camera", "index": 0, "name": "cam0"},
                2: {"id": 2, "type": "camera", "index": 1, "name": "cam1"},
                3: {"id": 3, "type": "imu", "index": 0, "name": "imu0"},
            },
            steps={
                11: {"type": "bundle_adjustment", "asset_group_signature": "1|"},
                14: {"type": "bundle_adjustment", "asset_group_signature": "2|"},
            },
        )
        camera_info = pd.DataFrame(
            [
                {"step_id": 6, "asset_id": 1, "camera_model": "double_sphere", "height": 512, "width": 512},
                {"step_id": 7, "asset_id": 2, "camera_model": "double_sphere", "height": 480, "width": 640},
            ]
        ).set_index(["step_id", "asset_id"], drop=False)
        intrinsic_data = "alpha = 0.5\ncx = 256.0\ncy = 256.0\nf = 160.0\nxi = 0.0"
        intrinsics = pd.DataFrame(
            [
                {"step_id": 11, "asset_id": 1, "camera_model": "double_sphere", "data": intrinsic_data},
                {"step_id": 14, "asset_id": 2, "camera_model": "double_sphere", "data": intrinsic_data},
            ]
        ).set_index(["step_id", "asset_id"])

        result = build_intrinsic_toml(
            workflow, {"camera_info": camera_info, "intrinsics": intrinsics}
        )

        self.assertIn("[workflow1.cam0]", result)
        self.assertIn("sensor_id = 'cam0'", result)
        self.assertIn("resolution = [512, 512]", result)
        self.assertIn("[workflow1.cam1]", result)
        self.assertIn("sensor_id = 'cam1'", result)
        self.assertIn("resolution = [480, 640]", result)

    def test_build_extrinsic_toml(self):
        workflow = Workflow(
            id=3,
            type="cam_imu",
            asset_group_signature="cam0_imu0",
            assets={
                1: {
                    "id": 1,
                    "type": "camera",
                    "index": 0,
                    "name": "frame_a_1",
                },
                2: {
                    "id": 2,
                    "type": "imu",
                    "index": 0,
                    "name": "frame_b_1",
                },
            },
            steps={
                5: {"type": "extrinsic_optimization", "asset_group_signature": "1|2|"},
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
