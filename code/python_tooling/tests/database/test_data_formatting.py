import os
import unittest
from tempfile import NamedTemporaryFile

import pandas as pd
import sqlite3
from database.sql_statement_loading import load_sql

from database.data_formatting import parse_workflows, process_workflow
from database.sql_table_loading import load_calibration_database


# Copy and pasted
def execute_sql(db_path, sql_query, params=()):
    with sqlite3.connect(db_path) as conn:
        cursor = conn.execute(sql_query, params)
        row = cursor.fetchone()

    # Because we use "RETURNING id" in some of our sql we need to try to consume the row otherwise the statement will
    # never finish.
    return row[0] if row is not None else None


def construct_test_db(db_path):
    # Enable foreign keys.
    execute_sql(db_path, "PRAGMA foreign_keys = ON;")

    # Metadata/workflow tables.
    execute_sql(db_path, load_sql("assets_table.sql"))
    execute_sql(db_path, load_sql("steps_table.sql"))
    execute_sql(db_path, load_sql("workflow_assets_table.sql"))
    execute_sql(db_path, load_sql("workflow_steps_table.sql"))
    execute_sql(db_path, load_sql("workflows_table.sql"))

    # Calibration artifact tables (only use a subset here to keep things simple).
    # execute_sql(db_path, load_sql("imu_data_table.sql"))
    # execute_sql(db_path, load_sql("images_table.sql"))

    # Make two workflows - one camera intrinsic and one camera-imu extrinsic calibration.
    cam_workflow_id = execute_sql(db_path, load_sql("workflows_insert.sql"), ("cam", "cam_signature"))
    cam_imu_workflow_id = execute_sql(db_path, load_sql("workflows_insert.sql"), ("cam_imu", "cam_imu_signature"))

    # Setup metadata/workflows - this does not reflect at all what actual calibration workflows would look like.
    camera_id = execute_sql(db_path, load_sql("assets_insert.sql"), ("camera", 0, ""))
    imu_id = execute_sql(db_path, load_sql("assets_insert.sql"), ("imu", 0, ""))

    # Add the assets to the workflows - note the camera belongs to both workflows.
    execute_sql(db_path, load_sql("workflow_assets_insert.sql"), (cam_workflow_id, camera_id))
    execute_sql(db_path, load_sql("workflow_assets_insert.sql"), (cam_imu_workflow_id, camera_id))
    execute_sql(db_path, load_sql("workflow_assets_insert.sql"), (cam_imu_workflow_id, imu_id))

    # Add two example steps to the database.
    image_loading_id = execute_sql(db_path, load_sql("steps_insert.sql"), ("image_loading", ""))
    imu_data_loading_id = execute_sql(db_path, load_sql("steps_insert.sql"), ("imu_data_loading", ""))

    # Add the steps to the workflows - note the image_loading belongs to both workflows.
    execute_sql(db_path, load_sql("workflow_steps_upsert.sql"), (cam_workflow_id, "image_loading", image_loading_id))
    execute_sql(db_path, load_sql("workflow_steps_upsert.sql"),
                (cam_imu_workflow_id, "image_loading", image_loading_id))
    execute_sql(db_path, load_sql("workflow_steps_upsert.sql"),
                (cam_imu_workflow_id, "imu_data_loading", imu_data_loading_id))


class TestDataFormatting(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        # TODO REMOVE - WE WANT ZERO DEP ON THE TEST DATA!
        self.db_path = os.getenv(
            "DB_PATH", "/temporary/code/test_data/dataset-calib-imu4_512_16.calib.db3"
        )

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

        self.assertEqual(len(workflows), 2)  # One asset and one step
        workflow_assert(
            workflows[0],
            1,
            "cam",
            "cam_signature",
            {"camera": {"id": 1, "index": 0, "name": ""}},
            {"image_loading": 1},
        )
        # Two assets and two steps
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
        db = load_calibration_database(self.db_path)
        workflows = parse_workflows(db)

        process_workflow(db, workflows[0])
