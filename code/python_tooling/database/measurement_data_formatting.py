from database.sql_table_loading import load_images_table
from database.types import SensorType


def process_images_table(table):
    if table is None:
        return None

    data = {}
    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            data[sensor_name] = {"measurements": {"images":{}}}

        timestamp_ns = int(row["timestamp_ns"])
        data[sensor_name]["measurements"]["images"][timestamp_ns] = row["data"]

    return data


def load_measurements(db_path):
    measurements = {SensorType.Camera: None, SensorType.Imu: None}

    table = load_images_table(db_path)
    if table is not None:
        measurements[SensorType.Camera] = process_images_table(table)

    return measurements
