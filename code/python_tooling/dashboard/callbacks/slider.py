from dash import MATCH, Input, Output, State

from dashboard.server import app
from dashboard.tools.selection import selected_targets
from dashboard.tools.playback import synchronized_targets
from dashboard.tools.workflow import stage_cameras


@app.callback(
    Output("selected-targets-store", "data"),
    Input("sensor-selection-dropdown", "value"),
    Input("step-selector", "value"),
    Input("workflow-data-store", "data"),
    Input("metadata-store", "data"),
    Input("stage-selector", "value"),
    Input("sync-max-offset", "value"),
)
def update_selected_targets(
    asset_id, step_id, workflow_data, metadata, stage_id, max_offset_ms
):
    if stage_id in ("multi_cam", "cam_imu"):
        return synchronized_targets(
            workflow_data,
            [camera["id"] for camera in stage_cameras(metadata, stage_id)],
            step_id,
            stage_id,
            max_offset_ms,
        )
    if asset_id is None:
        return []
    return selected_targets(workflow_data, asset_id, step_id)


@app.callback(
    Output({"type": "slider", "asset_id": MATCH, "sensor_type": MATCH}, "max"),
    Input({"type": "slider", "asset_id": MATCH, "sensor_type": MATCH}, "id"),
    Input("selected-targets-store", "data"),
)
def update_slider_properties(composite_id, targets):
    if not composite_id:
        return 0
    return max(0, len(targets or []) - 1)


@app.callback(
    Output({"type": "pause_button", "asset_id": MATCH}, "children"),
    Input({"type": "pause_button", "asset_id": MATCH}, "n_clicks"),
)
def update_pause_button_label(n_clicks):
    paused = (n_clicks or 0) % 2 == 1

    return "Play" if paused else "Pause"


app.clientside_callback(
    """
    function(frame_idx, targets) {
        const row = (targets || [])[frame_idx];
        return row ? row.timestamp_ns : "";
    }
    """,
    Output(
        {
            "type": "current_timestamp",
            "asset_id": MATCH,
            "sensor_type": MATCH,
        },
        "children",
    ),
    Input(
        {"type": "slider", "asset_id": MATCH, "sensor_type": MATCH},
        "value",
    ),
    Input("selected-targets-store", "data"),
)


app.clientside_callback(
    """
    function(interval, clicks, value, max) {
        if ((clicks || 0) % 2 === 1) { return dash_clientside.no_update; }
        if (value == null || max == null || value >= max) { return 0; }
        return value + 1;
    }
    """,
    Output({"type": "slider", "asset_id": MATCH, "sensor_type": MATCH}, "value"),
    Input("play-interval", "n_intervals"),
    Input({"type": "pause_button", "asset_id": MATCH}, "n_clicks"),
    State({"type": "slider", "asset_id": MATCH, "sensor_type": MATCH}, "value"),
    State({"type": "slider", "asset_id": MATCH, "sensor_type": MATCH}, "max"),
)


app.clientside_callback(
    "function(targets) { return 0; }",
    Output(
        {"type": "slider", "asset_id": MATCH, "sensor_type": MATCH},
        "value",
        allow_duplicate=True,
    ),
    Input("selected-targets-store", "data"),
    prevent_initial_call=True,
)
