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

        def workflow_assert(workflow, id, type, signature, assets, steps):
            self.assertEqual(workflow.id, id)
            self.assertEqual(workflow.type, type)
            self.assertEqual(workflow.signature, signature)
            self.assertEqual(workflow.assets, assets)
            self.assertEqual(workflow.steps, steps)

        self.assertEqual(len(workflows), 2)
        workflow_assert(
            workflows[0],
            1,
            "cam",
            "cam_signature",
            {"camera": {"id": 1, "index": 0, "name": ""}},
            {"image_loading": 1},
        )
        workflow_assert(
            workflows[1],
            2,
            "cam_imu",
            "cam_imu_signature",
            {
                "camera": {"id": 1, "index": 0, "name": ""},
                "imu": {"id": 2, "index": 0, "name": ""},
            },
            {"image_loading": 1, "imu_data_loading": 2},
        )

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
