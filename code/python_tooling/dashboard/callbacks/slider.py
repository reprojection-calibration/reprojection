from dash import MATCH, Input, Output, State, no_update

from dashboard.server import app
from dashboard.tools.selection import selected_targets


@app.callback(
    Output("selected-targets-store", "data"),
    Input("sensor-selection-dropdown", "value"),
    Input("step-selector", "value"),
    Input("workflow-data-store", "data"),
)
def update_selected_targets(asset_id, step_id, workflow_data):
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
    Output({"type": "slider", "asset_id": MATCH, "sensor_type": MATCH}, "value"),
    Input("play-interval", "n_intervals"),
    Input({"type": "pause_button", "asset_id": MATCH}, "n_clicks"),
    State({"type": "slider", "asset_id": MATCH, "sensor_type": MATCH}, "value"),
    State({"type": "slider", "asset_id": MATCH, "sensor_type": MATCH}, "max"),
)
def advance_slider(_, n_clicks, value, max_value):
    paused = (n_clicks or 0) % 2 == 1
    if paused:
        return no_update

    if value is None or max_value is None:
        return 0

    if value >= max_value:
        return 0
    else:
        return value + 1


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
