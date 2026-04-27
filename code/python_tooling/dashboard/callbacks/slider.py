from dash import MATCH, Input, Output, State, no_update

from dashboard.server import app
from database.types import SensorType


@app.callback(
    Output({"type": "slider", "sensor_name": MATCH, "sensor_type": MATCH}, "max"),
    Input({"type": "slider", "sensor_name": MATCH, "sensor_type": MATCH}, "id"),
    State("metadata-store", "data"),
)
def update_slider_properties(composite_id, metadata):
    if composite_id is None or metadata is None:
        return 0

    sensor_name = composite_id["sensor_name"]
    sensor_type = composite_id["sensor_type"]

    if sensor_type == SensorType.Camera:
        n_frames = metadata[sensor_name]["measurements"]["targets"]
    else:
        # TODO(Jack): Add the case for IMU when we need it!
        return 0

    return n_frames - 1


@app.callback(
    Output({"type": "slider", "sensor_name": MATCH, "sensor_type": MATCH}, "value"),
    Input("play-interval", "n_intervals"),
    Input({"type": "pause_button", "sensor_name": MATCH}, "n_clicks"),
    State({"type": "slider", "sensor_name": MATCH, "sensor_type": MATCH}, "value"),
    State({"type": "slider", "sensor_name": MATCH, "sensor_type": MATCH}, "max"),
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

    return "Play" if paused else "Pause"


app.clientside_callback(
    """
    function(composite_id, frame_idx, raw_data) {    
        if (!composite_id || frame_idx == null || !raw_data) {
            return dash_clientside.no_update;
        }
        
        const sensor_name = composite_id["sensor_name"];
        const sensor_type = composite_id["sensor_type"];
        
        let keys;
        if(sensor_type == "camera"){
            const targets = raw_data?.[sensor_name]?.measurements?.targets;
            if (!targets) {
                return dash_clientside.no_update;
            }
            
            keys = Object.keys(targets ?? {});
        } else{
            // TODO(Jack): Add logic for IMU case
            return dash_clientside.no_update;
        }

        const key = keys[frame_idx];
        if (!key) {
            throw new Error(`Invalid frame_idx: ${frame_idx}`);
        }
        
        return key;
    }
    """,
    Output(
        {
            "type": "current_timestamp",
            "sensor_name": MATCH,
            "sensor_type": MATCH,
        },
        "children",
    ),
    Input({"type": "slider", "sensor_name": MATCH, "sensor_type": MATCH}, "id"),
    Input(
        {"type": "slider", "sensor_name": MATCH, "sensor_type": MATCH},
        "value",
    ),
    State("raw-data-store", "data"),
)
