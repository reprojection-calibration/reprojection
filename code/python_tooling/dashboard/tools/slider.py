from dashboard.tools.time_handling import calculate_ticks_from_timestamps


def get_slider_properties(sensor, statistics, timestamps):
    if sensor not in statistics or sensor not in timestamps:
        return {}, 0

    tickvals_idx, _, ticktext = calculate_ticks_from_timestamps(timestamps[sensor])
    n_frames = statistics[sensor]["total_frames"]

    return dict(zip(tickvals_idx, ticktext)), max(n_frames - 1, 0)


# NOTE(Jack): Unfortunately the only way to achieve the parameterization of the clientside callbacks is to generate the
# code.
def make_slider_timestamp_clientside_callback(sensor_type):
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


def looping_increment(value, max_value):
    if value >= max_value:
        return 0

    return value + 1
