import unittest
from tempfile import NamedTemporaryFile

from dashboard.callbacks.metadata import update_result_summary, update_sensor_metadata
from dashboard.callbacks.populate_sensor_panel import render_sensor_panel
from dashboard.callbacks.slider import update_selected_targets
from dashboard.callbacks.workflow import update_stage_controls
from dashboard.tools.data_loading import load_database, refresh_sensor_list
from dashboard.tools.metadata import imu_step_selector_options, step_selector_options
from dashboard.tools.results import build_result_summary
from dashboard.tools.selection import selected_targets, table_rows
from dashboard.tools.workflow import build_stage_overview, stage_navigation
from tests.test_fixture import construct_staged_visualization_db


class TestWorkflowStages(unittest.TestCase):
    def setUp(self):
        with NamedTemporaryFile(suffix=".db3") as database:
            construct_staged_visualization_db(database.name)
            self.data, self.metadata = load_database(database.name, 1)
            self.camera_data, self.camera_metadata = load_database(database.name, 2)

    def test_stage_order_and_optional_stages(self):
        options, default = stage_navigation(self.metadata)
        self.assertEqual(
            [option["value"] for option in options],
            ["single_cam", "multi_cam", "cam_imu"],
        )
        self.assertEqual(default, "single_cam")
        self.assertFalse(any(option["disabled"] for option in options))
        options, _ = stage_navigation(self.camera_metadata)
        self.assertEqual(
            [option["disabled"] for option in options], [False, True, True]
        )
        options, default = stage_navigation(None)
        self.assertTrue(all(option["disabled"] for option in options))
        self.assertIsNone(default)

    def test_camera_results_are_scoped_to_stage_and_asset(self):
        self.assertEqual(step_selector_options(1, self.metadata, "single_cam")[1], 30)
        self.assertEqual(
            step_selector_options(1, self.metadata, "multi_cam"),
            (
                [
                    {"label": "Rig initialization", "value": 70},
                    {"label": "Rig optimization", "value": 71},
                ],
                71,
            ),
        )
        # Stereo camera 1 has reprojection errors but no rig poses.
        self.assertEqual(step_selector_options(2, self.metadata, "multi_cam")[1], 71)
        self.assertEqual(step_selector_options(1, self.metadata, "cam_imu")[1], 60)
        self.assertEqual(
            step_selector_options(2, self.metadata, "cam_imu"),
            ([{"label": "Visual-inertial optimization", "value": 60}], 60),
        )
        self.assertEqual(step_selector_options(1, self.metadata, "missing"), ([], None))

    def test_visual_inertial_shows_all_cameras_with_shared_imu(self):
        cameras, default = refresh_sensor_list(self.metadata, "cam_imu")
        self.assertEqual([camera["value"] for camera in cameras], [1, 2])
        self.assertEqual(default, 1)
        for asset_id in (1, 2):
            panel = render_sensor_panel(asset_id, self.metadata, "cam_imu")
            self.assertEqual(len(panel.children), 2)
            self.assertIn("IMU · imu", str(panel))
            targets = selected_targets(self.data, asset_id, 60)
            self.assertEqual({row["step_id"] for row in targets}, {19 + asset_id})
            errors = table_rows(
                self.data, "reprojection_errors", asset_id=asset_id, step_id=60
            )
            self.assertEqual(len(errors), 2)
            self.assertEqual({row["source_step_id"] for row in errors}, {19 + asset_id})
        self.assertIn(
            "Rig motion (reference camera)",
            str(render_sensor_panel(1, self.metadata, "cam_imu")),
        )
        self.assertEqual(
            table_rows(self.data, "camera_poses", asset_id=2, step_id=60), []
        )
        options, _ = stage_navigation(self.metadata)
        self.assertIn("2 cameras with results", str(options[2]["label"]))
        self.assertEqual(
            len(render_sensor_panel(1, self.metadata, "single_cam").children), 1
        )
        description = str(build_stage_overview("cam_imu", self.metadata))
        self.assertIn("all cameras", description)
        self.assertIn("reprojection errors", description)
        self.assertIn("Camera 0", description)

    def test_rig_layout_has_all_features_and_one_playback_control(self):
        def components(node):
            if isinstance(node, list):
                return [item for child in node for item in components(child)]
            if not hasattr(node, "to_plotly_json"):
                return []
            return [node] + components(getattr(node, "children", []))

        for stage in ("multi_cam", "cam_imu"):
            # The hidden camera selector must not determine which cameras are drawn.
            panel = render_sensor_panel(None, self.metadata, stage)
            nodes = components(panel)
            graphs = [
                node
                for node in nodes
                if isinstance(getattr(node, "id", None), dict)
                and node.id["type"] == "extracted_targets"
            ]
            self.assertEqual([graph.id["asset_id"] for graph in graphs], [1, 2])
            self.assertTrue(all(len(graph.figure.data) == 1 for graph in graphs))
            self.assertEqual(
                sum(
                    isinstance(getattr(node, "id", None), dict)
                    and node.id["type"] == "slider"
                    for node in nodes
                ),
                1,
            )
            self.assertEqual(update_stage_controls(stage)[0], {"display": "none"})
        mono = components(render_sensor_panel(2, self.metadata, "single_cam"))
        graph = next(
            node
            for node in mono
            if getattr(node, "id", None) == {"type": "extracted_targets", "asset_id": 2}
        )
        self.assertEqual(len(graph.figure.data), 2)

    def test_rig_selection_and_summary_include_every_camera(self):
        self.metadata["steps"].append(
            dict(step_id=61, type="visual_inertial_opt", asset_ids=[2, 4])
        )
        self.metadata["counts"].append(
            dict(step_id=61, asset_id=2, table="reprojection_errors", count=1)
        )
        _, options, value = update_sensor_metadata(
            1, self.metadata, self.data, "cam_imu"
        )
        self.assertEqual([option["value"] for option in options], [60, 61])
        self.assertEqual(value, 61)
        summary = update_result_summary(None, 60, self.metadata, self.data, "cam_imu")
        metrics = summary[0].children
        self.assertEqual(
            [metric.children[0].children for metric in metrics], ["2", "4"]
        )
        for stage, step in [("multi_cam", 71), ("cam_imu", 60)]:
            targets = update_selected_targets(
                None, step, self.data, self.metadata, stage, 20
            )
            self.assertEqual(set(targets[0]["cameras"]), {"1", "2"})

    def test_reference_only_results_remain_usable(self):
        self.metadata["counts"] = [
            row
            for row in self.metadata["counts"]
            if row["step_id"] != 60 or row["asset_id"] != 2
        ]
        self.assertEqual(step_selector_options(1, self.metadata, "cam_imu")[1], 60)
        self.assertEqual(step_selector_options(2, self.metadata, "cam_imu"), ([], None))
        options, _ = stage_navigation(self.metadata)
        self.assertIn("1 camera with results", str(options[2]["label"]))

    def test_imu_results_are_selected_independently_of_camera(self):
        self.metadata["steps"].append(
            {"step_id": 59, "type": "visual_inertial_init", "asset_ids": [1, 4]}
        )
        self.metadata["counts"].append(
            {"table": "imu_errors", "step_id": 59, "asset_id": 4, "count": 2}
        )
        options, default = imu_step_selector_options(4, self.metadata)
        self.assertIn({"label": "Visual-inertial initialization", "value": 59}, options)
        self.assertEqual(default, 60)
        self.assertNotIn(
            59,
            [o["value"] for o in step_selector_options(1, self.metadata, "cam_imu")[0]],
        )
        panel = render_sensor_panel(1, self.metadata, "cam_imu")
        selector = panel.children[1].children[1].children[1]
        self.assertEqual(selector.options, options)
        self.assertEqual(selector.value, default)
        self.assertEqual(step_selector_options(2, self.metadata, "cam_imu")[1], 60)
        self.assertNotIn(
            59,
            [
                o["value"]
                for o in step_selector_options(1, self.metadata, "single_cam")[0]
            ],
        )

        self.metadata["counts"] = [
            r for r in self.metadata["counts"] if r["step_id"] == 59
        ]
        self.assertEqual(step_selector_options(1, self.metadata, "cam_imu"), ([], None))
        self.assertEqual(imu_step_selector_options(4, self.metadata)[1], 59)
        self.assertEqual(imu_step_selector_options(99, self.metadata), ([], None))
        self.assertEqual(imu_step_selector_options(4, None), ([], None))

    def test_shared_results_and_missing_results_are_explained(self):
        result = str(build_result_summary(2, 71, self.metadata, self.data, "multi_cam"))
        self.assertIn("reference camera", result)
        self.assertIn("Relative sensor transforms", result)
        self.assertIn("Rig optimization", result)
        result = str(build_result_summary(2, 60, self.metadata, self.data, "cam_imu"))
        self.assertIn("reference camera", result)
        self.assertIn("Relative sensor transforms", result)
        self.assertIn("Visual-inertial optimization", result)
        partial = dict(self.metadata, counts=[], steps=[])
        self.assertEqual(step_selector_options(1, partial, "multi_cam"), ([], None))
        self.assertIn(
            "No pose or reprojection results",
            str(build_result_summary(1, None, partial, {}, "multi_cam")),
        )
        # Available assets keep an unfinished stage visible without claiming success.
        options, _ = stage_navigation(partial)
        self.assertFalse(options[1]["disabled"])
        self.assertIn("No pose or reprojection results yet", str(options[1]["label"]))
