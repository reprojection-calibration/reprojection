from dash import html

from dashboard.tools.selection import table_rows
from dashboard.tools.workflow import STEP_LABELS, stage_step_ids


def build_result_summary(asset_id, step_id, metadata, workflow_data, stage_id):
    if asset_id is None or not metadata:
        return []
    counts = {
        row["table"]: row["count"]
        for row in metadata["counts"]
        if row["asset_id"] == asset_id and row["step_id"] == step_id
    }
    content = []
    if step_id is None:
        content.append(
            html.P(
                "No pose or reprojection results recorded for this camera in this stage yet. You can inspect any available target detections below.",
                className="empty-state",
            )
        )
    else:
        metrics = [
            ("Pose samples", counts.get("camera_poses", 0)),
            ("Frames with reprojection errors", counts.get("reprojection_errors", 0)),
        ]
        content.append(
            html.Div(
                [
                    html.Div(
                        [html.Strong(f"{value:,}"), html.Span(label)],
                        className="result-metric",
                    )
                    for label, value in metrics
                ],
                className="result-metrics",
            )
        )
        if stage_id == "multi_cam" and not counts.get("camera_poses"):
            content.append(
                html.P(
                    "This camera has reprojection results. Select the reference camera to view the rig poses.",
                    className="stage-note",
                )
            )
    if stage_id in ("multi_cam", "cam_imu"):
        assets = {asset["id"]: asset for asset in metadata["assets"]}
        steps = {step["step_id"]: step for step in metadata["steps"]}
        allowed = (
            {step_id}
            if step_id is not None and stage_id == "multi_cam"
            else stage_step_ids(metadata, stage_id)
        )
        extrinsics = [
            row
            for row in table_rows(workflow_data, "extrinsics")
            if row["step_id"] in allowed
        ]
        if extrinsics:
            content.append(
                html.Details(
                    [
                        html.Summary("Relative sensor transforms"),
                        html.P(
                            "Each row maps the From frame into the To frame. Translation is in metres; rotation is an axis-angle vector in radians."
                        ),
                        html.Div(
                            html.Table(
                                [
                                    html.Thead(
                                        html.Tr(
                                            [
                                                html.Th(label)
                                                for label in (
                                                    "Result",
                                                    "From",
                                                    "To",
                                                    "x",
                                                    "y",
                                                    "z",
                                                    "rx",
                                                    "ry",
                                                    "rz",
                                                )
                                            ]
                                        )
                                    ),
                                    html.Tbody(
                                        [
                                            html.Tr(
                                                [
                                                    html.Td(
                                                        STEP_LABELS.get(
                                                            steps[row["step_id"]][
                                                                "type"
                                                            ],
                                                            steps[row["step_id"]][
                                                                "type"
                                                            ],
                                                        )
                                                    ),
                                                    html.Td(
                                                        assets[row["asset_b_id"]][
                                                            "name"
                                                        ]
                                                    ),
                                                    html.Td(
                                                        assets[row["asset_a_id"]][
                                                            "name"
                                                        ]
                                                    ),
                                                    *[
                                                        html.Td(f"{row[column]:.6g}")
                                                        for column in (
                                                            "x",
                                                            "y",
                                                            "z",
                                                            "rx",
                                                            "ry",
                                                            "rz",
                                                        )
                                                    ],
                                                ]
                                            )
                                            for row in extrinsics
                                        ]
                                    ),
                                ],
                                className="details-table",
                            ),
                            className="table-scroll",
                        ),
                    ],
                    open=True,
                    className="transform-details",
                )
            )
    return content
