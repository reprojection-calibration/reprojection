from dash import Input, Output, State

from dashboard.server import app
from dashboard.tools.time_handling import calculate_ticks_from_timestamps


# ERROR(Jack):
@app.callback(
    Output("frame-id-slider", "marks"),
    Output("frame-id-slider", "max"),
    Input("camera-sensor-dropdown", "value"),
    State("processed-data-store", "data"),
)
def update_slider_properties(sensor, processed_data):
    if sensor is None or processed_data is None:
        return {}, 0

    statistics, timestamps_sorted = processed_data
    if sensor not in statistics or sensor not in timestamps_sorted:
        return {}, 0

    timestamps_ns = timestamps_sorted[sensor]
    tickvals_idx, _, ticktext = calculate_ticks_from_timestamps(timestamps_ns)

    n_frames = statistics[sensor]["total_frames"]

    return dict(zip(tickvals_idx, ticktext)), max(n_frames - 1, 0)


# TODO(Jack): We need to display the exact nanosecond timestamp of the current frame somewhere and somehow. If this is
#  is the best way to do this I am not 100% sure just yet.
app.clientside_callback(
    """
    function(frame_idx, data, sensor) {
        if (!data || !sensor) {
            return "";
        }
    
        const timestamps = data[1][sensor]
        if (!timestamps || timestamps.length == 0 || timestamps.length <= frame_idx) {
            return "";
        }
    
        const timestamp_i = BigInt(timestamps[frame_idx])
    
        return timestamp_i.toString();
    }
    """,
    Output("slider-timestamp", "children"),
    Input("frame-id-slider", "value"),
    State("processed-data-store", "data"),
    State("camera-sensor-dropdown", "value"),
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


@app.callback(
    Output("frame-id-slider", "value"),
    Input("play-interval", "n_intervals"),
    State("frame-id-slider", "value"),
    State("frame-id-slider", "max"),
)
def advance_slider(_, value, max_value):
    if value is None:
        return 0
    if value >= max_value:
        return 0  # loop playback

    return value + 1
