import tomllib
import unittest
from textwrap import dedent

import numpy as np
import pandas as pd

from database.data_formatting import Workflow
from report.build_calibration_toml import build_extrinsic_toml, build_intrinsic_toml


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

        camera_info = pd.DataFrame(
            [
                {
                    "step_id": 1,
                    "asset_id": 1,
                    "camera_model": "pinhole_radtan4",
                    "height": 720,
                    "width": 1080,
                }
            ]
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
        )

        workflow_data = {
            "camera_info": camera_info,
            "intrinsics": camera_intrinsics,
        }

        result = build_intrinsic_toml(workflow, workflow_data)

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
                {
                    "step_id": 6,
                    "asset_id": 1,
                    "camera_model": "double_sphere",
                    "height": 512,
                    "width": 512,
                },
                {
                    "step_id": 7,
                    "asset_id": 2,
                    "camera_model": "double_sphere",
                    "height": 480,
                    "width": 640,
                },
            ]
        )
        intrinsic_data = "alpha = 0.5\ncx = 256.0\ncy = 256.0\nf = 160.0\nxi = 0.0"
        intrinsics = pd.DataFrame(
            [
                {
                    "step_id": 11,
                    "asset_id": 1,
                    "camera_model": "double_sphere",
                    "data": intrinsic_data,
                },
                {
                    "step_id": 14,
                    "asset_id": 2,
                    "camera_model": "double_sphere",
                    "data": intrinsic_data,
                },
            ]
        )

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
                5: {"type": "visual_inertial_opt", "asset_group_signature": "1|2|"},
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

        workflow_data = {
            "extrinsics": extrinsics,
        }

        result = build_extrinsic_toml(workflow, workflow_data)

        result_gt = """\
        [workflow3.extrinsic0]
        step_id = 5
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

    def test_extrinsics_export_only_final_optimized_sensor_pairs(self):
        workflow = Workflow(
            id=1,
            type="cam_imu",
            asset_group_signature="1|2|3|",
            assets={
                1: {"id": 1, "name": "cam0", "type": "camera"},
                2: {"id": 2, "name": "cam1", "type": "camera"},
                3: {"id": 3, "name": "imu", "type": "imu"},
            },
            steps={
                step: {"type": kind, "asset_group_signature": "1|2|3|"}
                for step, kind in [
                    (10, "stereo_rig_init"),
                    (11, "stereo_rig_opt"),
                    (12, "stereo_rig_opt"),
                    (20, "visual_inertial_init"),
                    (21, "visual_inertial_opt"),
                    (22, "visual_inertial_opt"),
                    (30, "visual_inertial_init"),
                    (31, "bundle_adjustment"),
                ]
            },
        )

        def row(step, a, b, x):
            return dict(
                step_id=step,
                asset_a_id=a,
                asset_b_id=b,
                rx=0,
                ry=0,
                rz=0,
                x=x,
                y=0,
                z=0,
            )

        extrinsics = pd.DataFrame(
            [
                row(22, 3, 1, 0.2),
                row(11, 2, 1, 9),
                row(10, 2, 1, 8),
                row(12, 2, 1, 0.1),
                row(20, 3, 1, 7),
                row(21, 3, 1, 6),
                row(12, 1, 1, 0),
                row(22, 3, 3, 0),
                row(30, 3, 1, 5),
                row(31, 1, 1, 0),
                # Distinct sensors may legitimately have an identity transform.
                row(22, 3, 2, 0),
                # An optimized result outside this workflow must not be exported.
                row(99, 3, 1, 4),
            ]
        )
        original = extrinsics.copy(deep=True)
        result = tomllib.loads(
            build_extrinsic_toml(workflow, {"extrinsics": extrinsics})
        )
        exported = result["workflow1"]
        self.assertEqual(list(exported), ["extrinsic0", "extrinsic1", "extrinsic2"])
        self.assertEqual(
            [(r["step_id"], r["frame_a"], r["frame_b"]) for r in exported.values()],
            [(12, "cam1", "cam0"), (22, "imu", "cam0"), (22, "imu", "cam1")],
        )
        self.assertEqual(exported["extrinsic0"]["tf_a_b"][0][3], 0.1)
        self.assertEqual(exported["extrinsic1"]["tf_a_b"][0][3], 0.2)
        np.testing.assert_allclose(exported["extrinsic2"]["tf_a_b"], np.eye(4))
        pd.testing.assert_frame_equal(extrinsics, original)

        for rows in [
            extrinsics.iloc[:0],
            pd.DataFrame([row(10, 2, 1, 8)]),
            pd.DataFrame([row(12, 1, 1, 0)]),
        ]:
            self.assertFalse(build_extrinsic_toml(workflow, {"extrinsics": rows}))
        self.assertFalse(build_extrinsic_toml(workflow, {}))
