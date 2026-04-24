from dash import MATCH, Input, Output, State, no_update

from dashboard.server import app


@app.callback(
    Output({"type": "slider", "sensor_name": MATCH}, "max"),
    Input({"type": "slider", "sensor_name": MATCH}, "id"),
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
    Output({"type": "slider", "sensor_name": MATCH}, "value"),
    Input("play-interval", "n_intervals"),
    Input({"type": "pause_button", "sensor_name": MATCH}, "n_clicks"),
    State({"type": "slider", "sensor_name": MATCH}, "value"),
    State({"type": "slider", "sensor_name": MATCH}, "max"),
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
    Output({"type": "pause_button", "sensor_name": MATCH}, "children"),
    Input({"type": "pause_button", "sensor_name": MATCH}, "n_clicks"),
)
def update_pause_button_label(n_clicks):
    paused = (n_clicks or 0) % 2 == 1

    return "▶" if paused else "⏸"
