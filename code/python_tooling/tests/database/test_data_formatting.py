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
                    {"id": 1, "type": "cam", "signature": "1|"},
                    {"id": 2, "type": "cam", "signature": "2|4|"},
                ]
            ),
            "assets": pd.DataFrame(
                [
                    {"type": "camera", "index": 0, "id": 1, "name": "/cam0/image_raw"},
                    {"type": "camera", "index": 1, "id": 2, "name": "/cam1/image_raw"},
                    {"type": "imu", "index": 0, "id": 3, "name": "/imu0"},
                ]
            ),
            "workflow_assets": pd.DataFrame(
                [
                    {"workflow_id": 1, "asset_id": 1},
                    {"workflow_id": 2, "asset_id": 2},
                    {"workflow_id": 2, "asset_id": 3},
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

        def workflow_assert(workflow, id, type, signature, assets, steps):
            self.assertEqual(workflow.id, id)
            self.assertEqual(workflow.type, type)
            self.assertEqual(workflow.signature, signature)
            self.assertEqual(list(workflow.assets), assets)
            self.assertEqual(list(workflow.steps), steps)

        self.assertEqual(len(workflows), 2)\
        # One asset and one step
        workflow_assert(workflows[0], 1, "cam", "1|",
                        [{'id': 1, 'index': 0, 'name': '/cam0/image_raw', 'type': 'camera'}],
                        [{'step_id': 1, 'type': 'image_loading'}])
        # Two assets and two steps
        workflow_assert(workflows[1], 2, "cam", "2|4|",
                        [{'id': 2, 'index': 1, 'name': '/cam1/image_raw', 'type': 'camera'},
                         {'id': 3, 'index': 0, 'name': '/imu0', 'type': 'imu'}],
                        [{'step_id': 2, 'type': 'image_loading'}, {'type': 'imu_data_loading', 'step_id': 3}])
