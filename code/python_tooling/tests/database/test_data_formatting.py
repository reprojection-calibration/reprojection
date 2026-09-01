import unittest
from tempfile import NamedTemporaryFile

import pandas as pd

from database.data_formatting import parse_workflows, process_workflow
from database.sql_table_loading import load_calibration_database
from tests.test_fixture import construct_test_db


class TestDataFormatting(unittest.TestCase):
    def test_parse_workflows(self):
        with NamedTemporaryFile(suffix=".db3") as tmp:
            construct_test_db(tmp.name)
            db = load_calibration_database(tmp.name)

        workflows = parse_workflows(db)

        def workflow_assert(workflow, id, type, asset_group_signature, assets, steps):
            self.assertEqual(workflow.id, id)
            self.assertEqual(workflow.type, type)
            self.assertEqual(workflow.asset_group_signature, asset_group_signature)
            self.assertEqual(workflow.assets, assets)
            self.assertEqual(workflow.steps, steps)

        self.assertEqual(len(workflows), 2)
        workflow_assert(
            workflows[0],
            1,
            "cam",
            "cam_signature",
            {1: {"id": 1, "type": "camera", "index": 0, "name": ""}},
            {1: {"type": "image_loading", "asset_group_signature": ""}},
        )
        workflow_assert(
            workflows[1],
            2,
            "cam_imu",
            "cam_imu_signature",
            {
                1: {"id": 1, "type": "camera", "index": 0, "name": ""},
                2: {"id": 2, "type": "imu", "index": 0, "name": ""},
            },
            {
                1: {"type": "image_loading", "asset_group_signature": ""},
                2: {"type": "imu_data_loading", "asset_group_signature": ""},
            },
        )

    def test_parse_multisensor_workflow_preserves_duplicate_types(self):
        db = {
            "workflows": pd.DataFrame(
                [{"id": 1, "type": "multi", "asset_group_signature": "1|2|"}]
            ),
            "workflow_assets": pd.DataFrame(
                [{"workflow_id": 1, "asset_id": 1}, {"workflow_id": 1, "asset_id": 2}]
            ),
            "assets": pd.DataFrame(
                [
                    {"id": 1, "type": "camera", "index": 0, "name": "cam0"},
                    {"id": 2, "type": "camera", "index": 1, "name": "cam1"},
                ]
            ),
            "workflow_steps": pd.DataFrame(
                [
                    {
                        "workflow_id": 1,
                        "step_id": 10,
                        "type": "image_loading",
                        "asset_group_signature": "1|",
                    },
                    {
                        "workflow_id": 1,
                        "step_id": 11,
                        "type": "image_loading",
                        "asset_group_signature": "2|",
                    },
                ]
            ),
        }

        workflow = parse_workflows(db)[0]
        self.assertEqual(
            [asset["name"] for asset in workflow.assets_of_type("camera")],
            ["cam0", "cam1"],
        )
        self.assertEqual(workflow.step_ids("image_loading"), [10, 11])

    def test_process_workflow(self):
        with NamedTemporaryFile(suffix=".db3") as tmp:
            construct_test_db(tmp.name)
            db = load_calibration_database(tmp.name)

        workflows = parse_workflows(db)

        expected = {
            "images_timestamps": pd.DataFrame(
                [
                    {
                        "step_id": 1,
                        "asset_id": 1,
                        "timestamp_ns": 0,
                    }
                ]
            ).set_index(["step_id", "asset_id"]),
            "imu_data": pd.DataFrame(
                [
                    {
                        "step_id": 2,
                        "asset_id": 2,
                        "timestamp_ns": 0,
                        "omega_x": 1.0,
                        "omega_y": 1.0,
                        "omega_z": 1.0,
                        "ax": 2.0,
                        "ay": 2.0,
                        "az": 2.0,
                    }
                ]
            ).set_index(["step_id", "asset_id"]),
        }

        workflow_data_0 = process_workflow(db, workflows[0])
        self.assertEqual(len(workflow_data_0), 1)
        pd.testing.assert_frame_equal(
            workflow_data_0["images_timestamps"],
            expected["images_timestamps"],
        )

        workflow_data_1 = process_workflow(db, workflows[1])
        self.assertEqual(len(workflow_data_1), 2)
        pd.testing.assert_frame_equal(
            workflow_data_1["images_timestamps"],
            expected["images_timestamps"],
        )
        pd.testing.assert_frame_equal(
            workflow_data_1["imu_data"],
            expected["imu_data"],
        )
