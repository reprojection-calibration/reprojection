import os
from enum import Enum

from database.calculate_metadata import count_data
from database.data_formatting import parse_workflows, process_workflow, to_legacy_data
from database.sql_table_loading import load_calibration_database


def refresh_database_list(db_dir):
    if db_dir is None or not os.path.exists(db_dir):
        return [], ""

    result = []
    for file_name in sorted(os.listdir(db_dir)):
        if not file_name.endswith(".calib.db3"):
            continue

        full_path = os.path.join(db_dir, file_name)
        result.append(
            {
                "label": file_name,
                "value": full_path,
            }
        )

    if len(result) == 0:
        return [], ""

    return result, result[0]["value"] if result else ""


def refresh_workflow_list(db_file):
    if db_file is None or not os.path.isfile(db_file):
        return [], ""

    db = load_calibration_database(db_file)
    workflows = parse_workflows(db)

    result = [
        {
            "label": f"{workflow.type} ({workflow.signature})",
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
        (
            workflow
            for workflow in workflows
            if workflow.id == workflow_id
        ),
        None,
    )

    if workflow is None:
        return {}, {}

    workflow_data = process_workflow(db, workflow)

    raw_data = to_legacy_data(workflow, workflow_data)
    metadata = count_data(raw_data)

    return raw_data, metadata


def refresh_sensor_list(metadata):
    if metadata is None:
        return [], ""

    result = []
    for sensor_name, value in metadata.items():
        sensor_type = value.get("type")
        if isinstance(sensor_name, Enum):
            sensor_type = sensor_type.name

        result.append(
            {
                "label": f"{sensor_name} ({sensor_type})",
                "value": sensor_name,
            }
        )

    return result, result[0]["value"] if result else ""
