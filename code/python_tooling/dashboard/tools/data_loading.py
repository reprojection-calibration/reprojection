import os


def refresh_database_list(dir):
    if dir is None or not os.path.exists(dir):
        return [], ""

    result = []
    for file_name in sorted(os.listdir(dir)):
        if not file_name.endswith(".db3"):
            continue

        full_path = os.path.join(dir, file_name)
        result.append(
            {
                "label": file_name,
                "value": full_path,
            }
        )

    if len(result) == 0:
        return [], ""

    return result, result[0]["value"] if result else ""


def refresh_sensor_list(metadata):
    if metadata is None:
        return [], ""

    result = []
    for sensor_name, value in metadata.items():
        sensor_type = value.get("type")

        result.append(f"{sensor_name} ({sensor_type.name})")

    return result, result[0] if result else ""
