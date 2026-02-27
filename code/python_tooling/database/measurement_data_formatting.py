from database.sql_table_loading import load_images_table, load_extracted_targets_table
from database.types import SensorType


def process_images_table(table):
    if table is None:
        return None

    data = {}
    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            data[sensor_name] = {"measurements": {"images": {}}}

        timestamp_ns = int(row["timestamp_ns"])
        data[sensor_name]["measurements"]["images"][timestamp_ns] = row["data"]

    return data


def process_extracted_targets_table(table, data):
    if table is None:
        return None

    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            raise KeyError(
                f"Error while loading extracted target for {sensor_name} - sensor does not already exist."
            )

        timestamp_ns = int(row["timestamp_ns"])
        if timestamp_ns not in data[sensor_name]["measurements"]["images"]:
            raise KeyError(
                f"Error while loading extracted target for {sensor_name} at time {timestamp_ns} - a corresponding image for the target was not found."
            )

        if "targets" not in data[sensor_name]["measurements"]:
            data[sensor_name]["measurements"].update({"targets": {}})

        target = row["data"]
        data[sensor_name]["measurements"]["targets"][timestamp_ns] = {
            "pixels": target["pixels"],
            "points": target["points"],
            "indices": target["indices"],
        }


# NOTE(Jack): The imu data only consists of one length six array so we store it timestamped directly under the
# 'measurements' key.
def process_imu_data_table(table):
    if table is None:
        return None

    data = {}
    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            data[sensor_name] = {"measurements": {}}

        timestamp_ns = int(row["timestamp_ns"])
        data[sensor_name]["measurements"][timestamp_ns] = row.iloc[-6:].tolist()

    return data


def load_measurements(db_path):
    measurements = {SensorType.Camera: None, SensorType.Imu: None}

    table = load_images_table(db_path)
    if table is not None:
        measurements[SensorType.Camera] = process_images_table(table)

    table = load_extracted_targets_table(db_path)
    if table is not None:
        load_extracted_targets_table(table, measurements[SensorType.Camera])

    return measurements
