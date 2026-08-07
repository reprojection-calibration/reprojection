import os
import unittest

import pandas as pd

from database.data_formatting import parse_workflows


class TestDataFormatting(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        # TODO REMOVE - WE WANT ZERO DEP ON THE TEST DATA!
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.calib.db3"
        )

    def test_parse_workflows(self):
        db = {
            "workflows": pd.DataFrame(
                [
                    {"id": 1, "type": "cam", "signature": "1|3|"},
                    {"id": 2, "type": "cam_imu", "signature": "2|3|4|"},
                ]
            ),
            "assets": pd.DataFrame(
                [
                    {"type": "camera", "index": 0, "id": 1, "name": "/cam0/image_raw"},
                    {"type": "camera", "index": 1, "id": 2, "name": "/cam1/image_raw"},
                    {"type": "target", "index": 0, "id": 3, "name": "aprilgrid"},
                    {"type": "imu", "index": 0, "id": 4, "name": "/imu0"},
                ]
            ),
            "workflow_assets": pd.DataFrame(
                [
                    {"workflow_id": 1, "asset_id": 1},
                    {"workflow_id": 1, "asset_id": 3},
                    {"workflow_id": 2, "asset_id": 2},
                    {"workflow_id": 2, "asset_id": 3},
                    {"workflow_id": 2, "asset_id": 4},
                ]
            ),
            "workflow_steps": pd.DataFrame(
                [
                    {"workflow_id": 1, "type": "image_loading", "step_id": 1},
                    {"workflow_id": 2, "type": "image_loading", "step_id": 2},
                    {"workflow_id": 2, "type": "imu_data_loading", "step_id": 3},
                ]
            ),
        }

        workflows = parse_workflows(db)

        def workflow_assert(workflow, id, type, signature, num_assets, num_steps):
            self.assertEqual(workflow.id, id)
            self.assertEqual(workflow.type, type)
            self.assertEqual(workflow.signature, signature)
            self.assertEqual(len(workflow.assets), num_assets)
            self.assertEqual(len(workflow.steps), num_steps)

        self.assertEqual(len(workflows), 2)
        workflow_assert(workflows[0], 1, "cam", "1|3|", 2, 1)
        workflow_assert(workflows[1], 2, "cam_imu", "2|3|4|", 3, 2)
