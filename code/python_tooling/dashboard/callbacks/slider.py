from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.time_handling import calculate_ticks_from_timestamps
from database.types import SensorType


def get_slider_properties(sensor, statistics, timestamps):
    if sensor not in statistics or sensor not in timestamps:
        return {}, 0

    tickvals_idx, _, ticktext = calculate_ticks_from_timestamps(timestamps[sensor])
    n_frames = statistics[sensor]["total_frames"]

    return dict(zip(tickvals_idx, ticktext)), max(n_frames - 1, 0)


def register_slider_properties_update_callback(
    slider_id, sensor_dropdown_id, sensor_type
):
    @app.callback(
        Output(slider_id, "marks"),
        Output(slider_id, "max"),
        Input(sensor_dropdown_id, "value"),
        State("processed-data-store", "data"),
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


# NOTE(Jack): Unfortunately the only way to achieve the parameterization of the clientside callbacks is to generate the
# code.
def make_slider_timestamps_clientside_callback(sensor_type):
    return f"""
    function(frame_idx, data, sensor) {{
        if (!data || !sensor) {{
            return "";
        }}

        const timestamps = data[1]["{sensor_type.value}"][sensor];
        if (!timestamps || timestamps.length == 0 || timestamps.length <= frame_idx) {{
            return "";
        }}

        const timestamp_i = BigInt(timestamps[frame_idx]);
        
        return timestamp_i.toString();
    }}
    """


def register_slider_timestamps_clientside_callback(
    timestamp_display_id, slider_id, sensor_dropdown_id, sensor_type
):
    app.clientside_callback(
        make_slider_timestamps_clientside_callback(sensor_type),
        Output(timestamp_display_id, "children"),
        Input(slider_id, "value"),
        State("processed-data-store", "data"),
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


def looping_increment(value, max_value):
    if value >= max_value:
        return 0

    return value + 1


# TODO(Jack): Why do we not use n_intervals directly? https://community.plotly.com/t/reset-n-intervals-in-dcc-interval/33078
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
