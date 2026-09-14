import unittest

import pandas as pd

from dashboard.tools.metadata import step_selector_options
from database.calculate_metadata import workflow_metadata
from database.data_formatting import Workflow


class TestCalculateMetadata(unittest.TestCase):
    def test_metadata_preserves_steps_and_asset_identity(self):
        workflow = Workflow(
            1,
            "multi_cam",
            "1|2|3|",
            {
                1: {"id": 1, "name": "same", "type": "camera"},
                2: {"id": 2, "name": "same", "type": "camera"},
                3: {"id": 3, "name": "board", "type": "target"},
            },
            {
                10: {"type": "bundle_adjustment", "asset_group_signature": "1|"},
                11: {"type": "bundle_adjustment", "asset_group_signature": "1|2|"},
                12: {"type": "target_info", "asset_group_signature": "3|"},
                13: {"type": "intrinsic_init", "asset_group_signature": "2|"},
                14: {"type": "stereo_rig_opt", "asset_group_signature": "1|2|"},
            },
        )
        tables = {
            "camera_poses": pd.DataFrame(
                [
                    {"step_id": 10, "asset_id": 1, "timestamp_ns": 100},
                    {"step_id": 11, "asset_id": 1, "timestamp_ns": 100},
                ]
            ),
            "target_info": pd.DataFrame([{"step_id": 12, "asset_id": 3}]),
            "intrinsics": pd.DataFrame([{"step_id": 13, "asset_id": 2}]),
            "reprojection_errors": pd.DataFrame(
                [{"step_id": 14, "asset_id": 2, "timestamp_ns": 100}]
            ),
            "extrinsics": pd.DataFrame(
                [{"step_id": 11, "asset_a_id": 1, "asset_b_id": 2}]
            ),
        }
        metadata = workflow_metadata(workflow, tables)
        self.assertEqual(metadata["assets"], list(workflow.assets.values()))
        self.assertEqual(len(metadata["counts"]), 7)
        self.assertEqual(
            [
                row["step_id"]
                for row in metadata["counts"]
                if row["table"] == "camera_poses"
            ],
            [10, 11],
        )
        self.assertEqual(
            step_selector_options(1, metadata)[0],
            [
                {"label": "Bundle adjustment (10)", "value": 10},
                {"label": "Bundle adjustment (11)", "value": 11},
            ],
        )
        # Reprojection errors alone qualify; intrinsics, extrinsics, and another
        # camera's poses do not, even when the selected asset belongs to the step.
        self.assertEqual(
            step_selector_options(2, metadata),
            ([{"label": "Rig optimization", "value": 14}], 14),
        )
        self.assertEqual(step_selector_options(3, metadata), ([], None))
