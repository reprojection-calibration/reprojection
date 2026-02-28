import os
from enum import Enum

from dash import html

from database.calculate_metadata import count_data, reference_timestamps
from database.data_formatting import load_data


def refresh_database_list(db_dir):
    if db_dir is None or not os.path.exists(db_dir):
        return [], ""

    result = []
    for file_name in sorted(os.listdir(db_dir)):
        if not file_name.endswith(".db3"):
            continue

        full_path = os.path.join(db_dir, file_name)
        result.append(
            {
                "label": file_name,
                "value": full_path,
            }
        )

    if len(result) == 0:
        return [], ""

    return result, result[0]["value"] if result else ""


def load_database(db_file):
    raw_data = load_data(db_file)
    metadata = count_data(raw_data)
    timestamps = reference_timestamps(raw_data)

    return raw_data, metadata, timestamps


def refresh_sensor_list(metadata):
    if metadata is None:
        return [], ""

    result = []
    for sensor_name, value in metadata.items():
        sensor_type = value.get("type")
        if isinstance(sensor_name, Enum):
            sensor_type = sensor_type.name

        result.append(
            {
                "label": f"{sensor_name} ({sensor_type})",
                "value": sensor_name,
            }
        )

    return result, result[0]["value"] if result else ""
