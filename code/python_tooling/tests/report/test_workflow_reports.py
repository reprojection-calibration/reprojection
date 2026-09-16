import unittest
from tempfile import NamedTemporaryFile

import numpy as np
import pandas as pd

from database.data_formatting import parse_workflows, process_workflow
from database.sql_table_loading import load_calibration_database
from report.build_calibration_toml import build_extrinsic_toml, build_intrinsic_toml
from report.build_pdf_report import build_camera_sections, build_imu_sections
from report.camera_figures import coverage_figure, error_figure
from report.dual_use_figures import measurement_delta_time_figures
from tests.test_fixture import construct_visualization_db


class TestWorkflowReports(unittest.TestCase):
    def test_reports_consume_loaded_tables(self):
        with NamedTemporaryFile(suffix=".db3") as tmp:
            construct_visualization_db(tmp.name)
            db = load_calibration_database(tmp.name)
        workflow = parse_workflows(db)[0]
        tables = process_workflow(db, workflow)
        camera_sections = build_camera_sections(workflow, tables)
        self.assertEqual(len(camera_sections), 2)
        for index, section in enumerate(camera_sections):
            errors = section["rows"][0][1]["fig"]
            np.testing.assert_allclose(errors.data[0].r, [index + 1, index + 1])
        self.assertEqual(len(build_imu_sections(workflow, tables)), 1)
        intrinsics = build_intrinsic_toml(workflow, tables)
        self.assertIn("[workflow1.cam0]", intrinsics)
        self.assertIn("[workflow1.cam1]", intrinsics)
        extrinsics = build_extrinsic_toml(workflow, tables)
        self.assertIn("step_id = 60", extrinsics)
        self.assertIn("frame_b = 'imu'", extrinsics)
        # Multiple frame pairs from one step must all reach the export.
        extra = tables["extrinsics"].copy()
        extra["asset_a_id"] = 2
        tables["extrinsics"] = pd.concat([tables["extrinsics"], extra])
        self.assertIn("[workflow1.extrinsic1]", build_extrinsic_toml(workflow, tables))

    def test_error_figure_joins_source_step_asset_and_timestamp(self):
        targets = pd.DataFrame(
            [
                dict(
                    step_id=10, asset_id=1, timestamp_ns=100, data={"pixels": [[0, 0]]}
                ),
                dict(
                    step_id=11,
                    asset_id=1,
                    timestamp_ns=100,
                    data={"pixels": [[50, 50], [60, 60]]},
                ),
                dict(
                    step_id=10,
                    asset_id=2,
                    timestamp_ns=100,
                    data={"pixels": [[70, 70], [80, 80]]},
                ),
            ]
        )
        errors = pd.DataFrame(
            [
                dict(
                    step_id=20,
                    source_step_id=10,
                    asset_id=1,
                    sample_timestamp_ns=100,
                    data=[[3, 4]],
                ),
            ]
        )
        figure = error_figure({"width": 100, "height": 100}, targets, errors)
        self.assertEqual(list(figure.data[0].r), [5])

    def test_coverage_without_camera_metadata(self):
        targets = pd.DataFrame([{"data": {"pixels": [[10, 20], [30, 40]]}}])
        figure = coverage_figure({}, targets)
        self.assertEqual(list(figure.layout.xaxis.range), [10, 30])

    def test_intervals_do_not_cross_asset_or_step_boundaries(self):
        rows = pd.DataFrame(
            [
                dict(step_id=step_id, asset_id=asset_id, timestamp_ns=timestamp)
                for step_id, asset_id in [(1, 1), (1, 2), (2, 1)]
                for timestamp in (1700000000000000001, 1700000000001000001)
            ]
        )
        figure, _ = measurement_delta_time_figures(rows)
        self.assertEqual(list(figure.data[0].y), [1, 1, 1])
