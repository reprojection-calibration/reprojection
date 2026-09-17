from dash import Input, Output

from dashboard.server import app
from dashboard.tools.workflow import build_stage_overview, stage_navigation


@app.callback(
    Output("stage-selector", "options"),
    Output("stage-selector", "value"),
    Input("metadata-store", "data"),
)
def update_stages(metadata):
    return stage_navigation(metadata)


@app.callback(
    Output("stage-overview", "children"),
    Input("stage-selector", "value"),
    Input("metadata-store", "data"),
)
def update_stage_overview(stage_id, metadata):
    return build_stage_overview(stage_id, metadata)


@app.callback(
    Output("camera-selection-control", "style"),
    Output("sync-controls", "style"),
    Input("stage-selector", "value"),
)
def update_stage_controls(stage_id):
    return (
        {"display": "none"} if stage_id in ("multi_cam", "cam_imu") else {},
        {} if stage_id == "cam_imu" else {"display": "none"},
    )
