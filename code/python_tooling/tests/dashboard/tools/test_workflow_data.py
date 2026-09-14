import json
import unittest
from tempfile import NamedTemporaryFile

import numpy as np
import pandas as pd

from dashboard.callbacks.populate_sensor_panel import render_sensor_panel
from dashboard.callbacks.slider import update_slider_properties
from dashboard.callbacks.timeseries_6d_figures import update_timeseries
from dashboard.tools.data_loading import (
    load_database,
    refresh_sensor_list,
    serialize_workflow,
)
from dashboard.tools.metadata import build_sensor_metadata_layout
from dashboard.tools.selection import selected_targets
from dashboard.tools.timeseries_6d import timeseries_6d_to_patch
from database.data_formatting import parse_workflows, process_workflow
from database.sql_table_loading import load_calibration_database
from tests.test_fixture import construct_visualization_db


def patch_values(patch):
    return {
        tuple(op["location"]): op["params"]["value"]
        for op in patch.to_plotly_json()["operations"]
    }


class TestWorkflowData(unittest.TestCase):
    def test_database_to_dashboard(self):
        with NamedTemporaryFile(suffix=".db3") as tmp:
            construct_visualization_db(tmp.name)
            data, metadata = load_database(tmp.name, 1)
            other, _ = load_database(tmp.name, 2)
        data = json.loads(json.dumps(data))
        metadata = json.loads(json.dumps(metadata))
        self.assertEqual(
            [option["value"] for option in refresh_sensor_list(metadata)[0]], [1, 2]
        )
        self.assertEqual(data["tables"]["target_info"][0]["asset_id"], 3)
        self.assertEqual(data["tables"]["extrinsics"][0]["step_id"], 60)
        self.assertEqual(len(other["tables"]["camera_poses"]), 2)
        self.assertEqual(
            {row["asset_id"] for row in other["tables"]["camera_poses"]}, {2}
        )
        cards = build_sensor_metadata_layout(1, metadata, data)
        self.assertIn("Target board (3, step 42)", str(cards))
        self.assertIn("camera_model", str(cards))
        targets = selected_targets(data, 1, 30)
        self.assertEqual(
            [row["timestamp_ns"] for row in targets],
            ["1700000000000000001", "1700000000000000002"],
        )
        self.assertEqual({row["step_id"] for row in targets}, {20})
        self.assertEqual(update_slider_properties({"asset_id": 1}, targets), 1)
        self.assertEqual(update_slider_properties({"asset_id": 1}, []), 0)
        self.assertEqual(
            render_sensor_panel(1, metadata)
            .children[0]
            .children[0]
            .children[0]
            .children,
            "Camera 0 · same",
        )
        patch = patch_values(
            update_timeseries({"asset_id": 1, "sensor_type": "camera"}, 30, data)
        )
        self.assertEqual(
            patch[("data", 0, "x")], [1700000000000000001, 1700000000000000002]
        )
        np.testing.assert_allclose(patch[("data", 3, "y")], [-1, -1])
        self.assertEqual(data["tables"]["camera_poses"][0]["x"], 1)
        empty = patch_values(
            update_timeseries({"asset_id": 1, "sensor_type": "camera"}, 40, data)
        )
        self.assertEqual(empty[("data", 0, "x")], [])

    def test_same_camera_and_timestamp_at_different_steps_survive(self):
        with NamedTemporaryFile(suffix=".db3") as tmp:
            construct_visualization_db(tmp.name)
            db = load_calibration_database(tmp.name)
        workflow = parse_workflows(db)[0]
        # Another group can contain the same sensor and produce the same step type.
        workflow.steps[22] = {
            "type": "feature_extraction",
            "asset_group_signature": "1|3|",
        }
        workflow.steps[32] = {
            "type": "bundle_adjustment",
            "asset_group_signature": "1|3|",
        }
        for name, step_id, source_step_id in [
            ("extracted_targets", 22, 10),
            ("camera_poses", 32, 22),
            ("reprojection_errors", 32, 22),
        ]:
            extra = db[name].loc[db[name]["asset_id"] == 1].copy()
            extra["step_id"] = step_id
            extra["source_step_id"] = source_step_id
            db[name] = pd.concat([db[name], extra], ignore_index=True)
        data = serialize_workflow(workflow, process_workflow(db, workflow))
        self.assertEqual(len(data["tables"]["camera_poses"]), 6)
        self.assertEqual(
            {row["step_id"] for row in selected_targets(data, 1, 32)}, {22}
        )
        self.assertEqual(
            {row["step_id"] for row in selected_targets(data, 1, 30)}, {20}
        )

    def test_loading_and_result_steps_select_their_source_measurements(self):
        data = {
            "tables": {
                "extracted_targets": [
                    {"step_id": 20, "source_step_id": 10, "asset_id": 1},
                    {"step_id": 21, "source_step_id": 11, "asset_id": 1},
                ],
                "imu_data": [
                    dict(
                        step_id=step,
                        asset_id=4,
                        timestamp_ns="100",
                        omega_x=step,
                        omega_y=0,
                        omega_z=0,
                        ax=0,
                        ay=0,
                        az=0,
                    )
                    for step in (50, 51)
                ],
                "imu_errors": [
                    dict(
                        step_id=60,
                        source_step_id=51,
                        asset_id=4,
                        timestamp_ns="100",
                        omega_x=1,
                        omega_y=0,
                        omega_z=0,
                        ax=0,
                        ay=0,
                        az=0,
                    )
                ],
            }
        }
        self.assertEqual(
            [row["step_id"] for row in selected_targets(data, 1, 11)], [21]
        )
        for step, expected in [(50, 50), (60, 51)]:
            patch = patch_values(
                update_timeseries({"asset_id": 4, "sensor_type": "imu"}, step, data)
            )
            np.testing.assert_allclose(patch[("data", 0, "y")], [expected])

    def test_imu_errors_join_full_source_key_and_clear(self):
        def row(step_id, timestamp, value, **extra):
            return dict(
                step_id=step_id,
                asset_id=4,
                timestamp_ns=timestamp,
                omega_x=value,
                omega_y=value,
                omega_z=value,
                ax=value,
                ay=value,
                az=value,
                **extra
            )

        data = [row(50, "101", 1), row(51, "101", 2), row(50, "102", 3)]
        errors = [row(60, "101", -5, source_step_id=51)]
        patch = patch_values(timeseries_6d_to_patch(data, errors=errors))
        np.testing.assert_allclose(
            patch[("data", 0, "error_y")]["arrayminus"], [np.nan, 5, np.nan]
        )
        patch = patch_values(timeseries_6d_to_patch(data))
        self.assertFalse(patch[("data", 0, "error_y")]["visible"])
