import os

from dashboard.tools.workflow import camera_label, stage_cameras
from database.calculate_metadata import workflow_metadata
from database.data_formatting import parse_workflows, process_workflow
from database.sql_table_loading import load_calibration_database


def refresh_workflow_list(db_file):
    if db_file is None or not os.path.isfile(db_file):
        return [], ""

    db = load_calibration_database(db_file)
    workflows = parse_workflows(db)

    result = [
        {
            "label": f"Workflow {workflow.id} · {len(workflow.assets_of_type("camera"))} cameras"
            + (" + IMU" if workflow.assets_of_type("imu") else ""),
            "value": workflow.id,
        }
        for workflow in workflows
    ]

    return result, result[0]["value"] if result else ""


def load_database(db_file, workflow_id):
    if db_file is None or workflow_id is None:
        return {}, {}

    db = load_calibration_database(db_file)
    workflows = parse_workflows(db)

    workflow = next(
        (workflow for workflow in workflows if workflow.id == workflow_id),
        None,
    )

    if workflow is None:
        return {}, {}

    workflow_data = process_workflow(db, workflow)

    return serialize_workflow(workflow, workflow_data), workflow_metadata(
        workflow, workflow_data
    )


def serialize_workflow(workflow, tables):
    """JSON transport only: retain SQL rows, with lossless timestamp strings."""
    records = {}
    for name, table in tables.items():
        table = table.copy()
        timestamp_column = (
            "sample_timestamp_ns" if name == "reprojection_errors" else "timestamp_ns"
        )
        if timestamp_column in table:
            table = table.sort_values([timestamp_column, "step_id", "asset_id"])
            table[timestamp_column] = table[timestamp_column].map(str)
        records[name] = table.to_dict("records")
    return {
        "id": workflow.id,
        "type": workflow.type,
        "asset_group_signature": workflow.asset_group_signature,
        "assets": list(workflow.assets.values()),
        "steps": [
            dict(step_id=step_id, **step) for step_id, step in workflow.steps.items()
        ],
        "tables": records,
    }


def refresh_sensor_list(metadata, stage_id="single_cam"):
    result = [
        {"label": camera_label(camera), "value": camera["id"]}
        for camera in stage_cameras(metadata, stage_id)
    ]
    return result, result[0]["value"] if result else None
