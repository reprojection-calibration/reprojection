from dash import MATCH, Input, Output, State

from dashboard.server import app


@app.callback(
    Output({"type": "target_slider", "sensor_name": MATCH}, "max"),
    Input({"type": "target_slider", "sensor_name": MATCH}, "id"),
    State("metadata-store", "data"),
)
def update_target_slider_properties(composite_id, metadata):
    if composite_id is None or metadata is None:
        return 0

    sensor_name = composite_id["sensor_name"]
    try:
        n_targets = metadata[sensor_name]["measurements"]["targets"]
    except (KeyError, TypeError):
        return 0

    return n_targets - 1


@app.callback(
    Output({"type": MATCH, "sensor_name": MATCH}, "value"),
    Input("play-interval", "n_intervals"),
    State({"type": MATCH, "sensor_name": MATCH}, "value"),
    State({"type": MATCH, "sensor_name": MATCH}, "max"),
)
def advance_slider(_, value, max_value):
    if value is None or max_value is None:
        return 0

    if value >= max_value:
        return 0
    else:
        return value + 1
