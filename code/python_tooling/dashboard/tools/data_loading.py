def refresh_sensor_list(metadata):
    if metadata is None:
        return [], ""

    result = []
    for sensor_name, value in metadata.items():
        sensor_type = value.get("type")

        result.append(f"{sensor_name} ({sensor_type.name})")

    return result, result[0] if result else ""
