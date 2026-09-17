from dash import Input, Output, html

from dashboard.server import app
from dashboard.tools.metadata import (
    build_sensor_metadata_layout,
    step_selector_options,
    result_selector_options,
)
from dashboard.tools.results import build_result_summary
from dashboard.tools.workflow import stage_cameras, result_step_ids, camera_label


@app.callback(
    Output("sensor-statistics-container", "children"),
    Output("step-selector", "options"),
    Output("step-selector", "value"),
    Input("sensor-selection-dropdown", "value"),
    Input("metadata-store", "data"),
    Input("workflow-data-store", "data"),
    Input("stage-selector", "value"),
)
def update_sensor_metadata(asset_id, metadata, workflow_data, stage_id):
    if stage_id in ("multi_cam", "cam_imu"):
        cameras = stage_cameras(metadata, stage_id)
        available = set().union(
            *(result_step_ids(metadata, camera["id"], stage_id) for camera in cameras)
        )
        options, value = result_selector_options(metadata or {}, available)
        return (
            [
                html.Div(
                    [
                        html.H3(camera_label(camera)),
                        *build_sensor_metadata_layout(
                            camera["id"], metadata, workflow_data
                        ),
                    ]
                )
                for camera in cameras
            ],
            options,
            value,
        )
    options, value = step_selector_options(asset_id, metadata, stage_id)
    return (
        build_sensor_metadata_layout(asset_id, metadata, workflow_data),
        options,
        value,
    )


@app.callback(
    Output("result-summary", "children"),
    Input("sensor-selection-dropdown", "value"),
    Input("step-selector", "value"),
    Input("metadata-store", "data"),
    Input("workflow-data-store", "data"),
    Input("stage-selector", "value"),
)
def update_result_summary(asset_id, step_id, metadata, workflow_data, stage_id):
    return build_result_summary(
        asset_id,
        step_id,
        metadata,
        workflow_data,
        stage_id,
        all_cameras=stage_id in ("multi_cam", "cam_imu"),
    )
