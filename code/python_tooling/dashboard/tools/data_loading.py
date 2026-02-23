def refresh_sensor_list(metadata, sensor_type):
    if metadata is None:
        return [], ""
    statistics, _ = metadata

    statistics = statistics[sensor_type]
    if statistics is None:
        return [], ""

    sensor_names = sorted(statistics.keys())

    if len(sensor_names) == 0:
        return [], ""

    return sensor_names, sensor_names[0]
