def refresh_sensor_list(metadata, sensor_type):
    if metadata is None:
        return [], ""
    statistics, _ = metadata

    # TODO(Jack): Do we really need to check this in every single consuming function? I think we can count on the fact
    #  that it is there.
    if sensor_type not in statistics:
        return [], ""

    sensor_statistics = statistics[sensor_type]
    sensor_names = sorted(sensor_statistics.keys())

    if len(sensor_names) == 0:
        return [], ""

    return sensor_names, sensor_names[0]
