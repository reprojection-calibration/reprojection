import unittest
from tempfile import NamedTemporaryFile

from dashboard.callbacks.populate_sensor_panel import render_sensor_panel
from dashboard.tools.data_loading import load_database, refresh_sensor_list
from dashboard.tools.metadata import step_selector_options
from dashboard.tools.results import build_result_summary
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
        self.assertEqual(step_selector_options(2, self.metadata, "cam_imu"), ([], None))
        self.assertEqual(step_selector_options(1, self.metadata, "missing"), ([], None))

    def test_visual_inertial_shows_reference_camera_and_imu_together(self):
        cameras, default = refresh_sensor_list(self.metadata, "cam_imu")
        self.assertEqual([camera["value"] for camera in cameras], [1])
        self.assertEqual(default, 1)
        panel = render_sensor_panel(1, self.metadata, "cam_imu")
        self.assertEqual(len(panel.children), 2)
        self.assertIn("IMU · imu", str(panel))
        self.assertEqual(
            len(render_sensor_panel(1, self.metadata, "single_cam").children), 1
        )
        description = str(build_stage_overview("cam_imu", self.metadata))
        self.assertIn("individual camera calibration", description)
        self.assertIn("Camera 0", description)

    def test_shared_results_and_missing_results_are_explained(self):
        result = str(build_result_summary(2, 71, self.metadata, self.data, "multi_cam"))
        self.assertIn("reference camera", result)
        self.assertIn("Relative sensor transforms", result)
        self.assertIn("Rig optimization", result)
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
