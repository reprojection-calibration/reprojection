from dash import html

from dashboard.tools.selection import table_rows
from dashboard.tools.workflow import STAGES, STEP_LABELS, result_step_ids


def extract_labeled_metadata(data, parent_keys=None):
    if parent_keys is None:
        parent_keys = []

    statistics = []

    for key, value in data.items():
        current_path = parent_keys + [str(key)]

        if isinstance(value, dict):
            statistics.extend(extract_labeled_metadata(value, current_path))
        else:
            statistics.append((current_path, value))

    return statistics


def build_sensor_statistics_html(sensor_metadata):
    return [
        html.Table(
            [
                html.Thead(
                    html.Tr([html.Th("Source"), html.Th("Property"), html.Th("Value")])
                ),
                html.Tbody(
                    [
                        html.Tr(
                            [
                                html.Td(path[0]),
                                html.Td(" / ".join(path[1:])),
                                html.Td(str(value)),
                            ]
                        )
                        for path, value in extract_labeled_metadata(sensor_metadata)
                    ]
                ),
            ],
            className="details-table",
        )
    ]


def step_selector_options(asset_id, metadata, stage_id=None):
    if asset_id is None or not metadata:
        return [], None
    available = result_step_ids(metadata, asset_id, stage_id)
    return result_selector_options(metadata, available)


def imu_step_selector_options(asset_id, metadata):
    available = {
        row["step_id"]
        for row in (metadata or {}).get("counts", [])
        if row["asset_id"] == asset_id
        and row["table"] == "imu_errors"
        and row["count"] > 0
    }
    return result_selector_options(metadata or {}, available)


def result_selector_options(metadata, available):
    order = {
        step_type: index
        for index, step_type in enumerate(
            step_type for stage in STAGES for step_type in stage.steps
        )
    }
    steps = sorted(
        (step for step in metadata.get("steps", []) if step["step_id"] in available),
        key=lambda step: (order.get(step["type"], len(order)), step["step_id"]),
    )
    options = [
        {
            "label": STEP_LABELS.get(step["type"], step["type"])
            + (
                f" ({step['step_id']})"
                if sum(other["type"] == step["type"] for other in steps) > 1
                else ""
            ),
            "value": step["step_id"],
        }
        for step in steps
    ]
    # Open the final available result; earlier results remain available for comparison.
    return options, options[-1]["value"] if options else None


def build_sensor_metadata_layout(asset_id, metadata, workflow_data):
    if asset_id is None or not metadata:
        return []

    statistics = {}
    for row in table_rows(workflow_data, "camera_info", asset_id=asset_id):
        statistics[f"camera_info (step {row['step_id']})"] = {
            key: value
            for key, value in row.items()
            if key not in ("step_id", "asset_id")
        }
    # Target descriptions belong to workflow target assets, not the selected camera.
    assets = {asset["id"]: asset for asset in metadata["assets"]}
    for row in table_rows(workflow_data, "target_info"):
        target = assets[row["asset_id"]]
        label = f"Target {target['name']} ({target['id']}, step {row['step_id']})"
        statistics[label] = {
            key: value
            for key, value in row.items()
            if key not in ("step_id", "asset_id")
        }
    return build_sensor_statistics_html(statistics)
