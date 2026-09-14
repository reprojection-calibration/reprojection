import unittest
import json

from dashboard.app import app


def component_ids(node):
    if isinstance(node, list):
        return [identifier for child in node for identifier in component_ids(child)]
    if not isinstance(node, dict):
        return []
    props = node.get("props", {})
    identifiers = [props["id"]] if "id" in props else []
    return identifiers + component_ids(props.get("children"))


class TestDashboardLayout(unittest.TestCase):
    def setUp(self):
        self.client = app.server.test_client()

    def test_fixed_callback_inputs_exist_at_startup(self):
        layout = self.client.get("/_dash-layout").get_json()
        ids = component_ids(layout)
        self.assertEqual(ids.count("step-selector"), 1)
        callbacks = self.client.get("/_dash-dependencies").get_json()
        for callback in callbacks:
            for dependency in callback["inputs"]:
                identifier = dependency["id"]
                if not identifier.startswith("{"):
                    self.assertIn(identifier, ids)

    def test_metadata_updates_selector_properties_when_loading_and_clearing(self):
        callback_id = next(
            key
            for key in app.callback_map
            if "sensor-statistics-container.children" in key
        )
        outputs = [
            output.to_dict() for output in app.callback_map[callback_id]["output"]
        ]
        metadata = {
            "assets": [{"id": 1, "name": "camera", "type": "camera"}],
            "counts": [
                {"table": "camera_poses", "step_id": 10, "asset_id": 1, "count": 1}
            ],
            "steps": [{"step_id": 10, "type": "bundle_adjustment", "asset_ids": [1]}],
        }
        for asset_id, stored_metadata, expected_value in [
            (None, None, None),
            (1, metadata, 10),
            (None, {}, None),
        ]:
            response = self.client.post(
                "/_dash-update-component",
                json={
                    "output": callback_id,
                    "outputs": outputs,
                    "inputs": [
                        {
                            "id": "sensor-selection-dropdown",
                            "property": "value",
                            "value": asset_id,
                        },
                        {
                            "id": "metadata-store",
                            "property": "data",
                            "value": stored_metadata,
                        },
                        {"id": "workflow-data-store", "property": "data", "value": {}},
                        {
                            "id": "stage-selector",
                            "property": "value",
                            "value": "single_cam",
                        },
                    ],
                    "state": [],
                    "changedPropIds": ["metadata-store.data"],
                },
            )
            self.assertEqual(response.status_code, 200)
            updates = response.get_json()["response"]
            self.assertEqual(updates["step-selector"]["value"], expected_value)
            self.assertEqual(
                updates["step-selector"]["options"],
                (
                    [{"label": "Bundle adjustment", "value": 10}]
                    if expected_value is not None
                    else []
                ),
            )
            self.assertNotIn(
                "step-selector",
                component_ids(updates["sensor-statistics-container"]["children"]),
            )

    def test_imu_callback_uses_local_selection_and_updates_residuals(self):
        callback_id = next(
            key for key in app.callback_map
            if '"sensor_type":"imu"' in key and 'timeseries' in key
        )
        dependencies = app.callback_map[callback_id]["inputs"]
        self.assertNotIn("step-selector", [item["id"] for item in dependencies])
        selector_id = {"type": "imu-step-selector", "asset_id": 4}
        graph_id = {"type": "timeseries", "asset_id": 4, "sensor_type": "imu"}
        measurement = dict(step_id=3, asset_id=4, timestamp_ns="100",
                           omega_x=1, omega_y=2, omega_z=3, ax=4, ay=5, az=6)
        data = {"tables": {
            "imu_data": [measurement],
            "imu_errors": [dict(measurement, step_id=step, source_step_id=3, omega_x=error)
                           for step, error in [(18, -0.5), (19, -0.1)]],
        }}
        for step, error in [(18, 0.5), (19, 0.1), (None, None)]:
            response = self.client.post("/_dash-update-component", json={
                "output": callback_id,
                "outputs": {"id": graph_id, "property": "figure"},
                "inputs": [
                    {"id": json.dumps(selector_id, sort_keys=True, separators=(",", ":")),
                     "property": "id", "value": selector_id},
                    {"id": json.dumps(selector_id, sort_keys=True, separators=(",", ":")),
                     "property": "value", "value": step},
                    {"id": "workflow-data-store", "property": "data", "value": data},
                ],
                "state": [],
                "changedPropIds": [json.dumps(selector_id, sort_keys=True, separators=(",", ":")) + ".value"],
            })
            self.assertEqual(response.status_code, 200)
            figure = next(iter(response.get_json()["response"].values()))["figure"]
            values = {tuple(op["location"]): op["params"]["value"]
                      for op in figure["operations"]}
            self.assertEqual(values[("data", 0, "y")], [1])
            residual = values[("data", 0, "error_y")]
            self.assertEqual(residual["visible"], error is not None)
            if error is not None:
                self.assertEqual(residual["arrayminus"], [error])
