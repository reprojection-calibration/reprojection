from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.slider import (
    get_slider_properties,
    looping_increment,
    make_slider_timestamps_clientside_callback,
)
from database.types import SensorType


@app.callback(
    Output("play-interval", "disabled"),
    Output("play-button", "children"),
    Input("play-button", "n_clicks"),
)
def toggle_play(n_clicks):
    # We play by default (n_clicks=0), which means that only when we have an odd number of clicks is "playing" false.
    playing = n_clicks % 2 == 0

    # If we want to play then we do NOT want the play interval disabled and we want the button to display the pause
    # graphic and message so the user knows to click there to pause. If we are already paused then the opposite is true.
    if playing:
        return False, "⏸ Pause"
    else:
        return True, "▶ Play"


def register_slider_properties_update_callback(
    slider_id, sensor_dropdown_id, sensor_type
):
    @app.callback(
        Output(slider_id, "marks"),
        Output(slider_id, "max"),
        Input(sensor_dropdown_id, "value"),
        State("metadata-store", "data"),
    )
    def update_slider_properties(sensor, metadata):
        if sensor is None or metadata is None:
            return {}, 0
        statistics, timestamps = metadata

        return get_slider_properties(
            sensor,
            statistics[sensor_type],
            timestamps[sensor_type],
        )


register_slider_properties_update_callback(
    "camera-frame-id-slider", "camera-sensor-dropdown", SensorType.Camera
)
register_slider_properties_update_callback(
    "imu-frame-id-slider", "imu-sensor-dropdown", SensorType.Imu
)


def register_slider_timestamps_clientside_callback(
    timestamp_display_id, slider_id, sensor_dropdown_id, sensor_type
):
    app.clientside_callback(
        make_slider_timestamps_clientside_callback(sensor_type),
        Output(timestamp_display_id, "children"),
        Input(slider_id, "value"),
        State("metadata-store", "data"),
        State(sensor_dropdown_id, "value"),
    )


register_slider_timestamps_clientside_callback(
    "camera-timestamp-display",
    "camera-frame-id-slider",
    "camera-sensor-dropdown",
    SensorType.Camera,
)

register_slider_timestamps_clientside_callback(
    "imu-timestamp-display",
    "imu-frame-id-slider",
    "imu-sensor-dropdown",
    SensorType.Imu,
)


def register_slider_advance_callback(slider_id):
    @app.callback(
        Output(slider_id, "value"),
        Input("play-interval", "n_intervals"),
        State(slider_id, "value"),
        State(slider_id, "max"),
    )
    def advance_slider(_, value, max_value):
        if value is None or max_value is None:
            return 0

        return looping_increment(value, max_value)


register_slider_advance_callback("camera-frame-id-slider")
register_slider_advance_callback("imu-frame-id-slider")
