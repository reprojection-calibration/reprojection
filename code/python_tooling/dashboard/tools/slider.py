from dashboard.tools.time_handling import calculate_ticks_from_timestamps


def update_slider_properties(sensor, metadata, sensor_type):
    if sensor is None or metadata is None:
        return {}, 0
    statistics, timestamps = metadata

    statistics = statistics[sensor_type]
    timestamps = timestamps[sensor_type]

    if statistics is None or timestamps is None:
        return {}, 0

    if sensor not in statistics or sensor not in timestamps:
        return {}, 0

    tickvals_idx, _, ticktext = calculate_ticks_from_timestamps(timestamps[sensor])
    n_frames = statistics[sensor]["total_frames"]

    return dict(zip(tickvals_idx, ticktext)), max(n_frames - 1, 0)


# NOTE(Jack): Unfortunately the only way to achieve the parameterization of the clientside callbacks is to generate the
# code.
def make_slider_timestamp_clientside_callback(sensor_type):
    return f"""
    function(frame_idx, metadata, sensor) {{
        if (frame_idx == null || !metadata || !sensor) {{
            return "";
        }}
        
        const timestamp_result = window.dataInputUtils.getTimestamps(metadata, "{sensor_type.value}", sensor, frame_idx);
        if (!timestamp_result) {{
            return "";
        }}
        const {{_, timestamp_i}} = timestamp_result;
        
        return timestamp_i.toString();
    }}
    """


def looping_increment(value, max_value):
    if value >= max_value:
        return 0

    return value + 1
