import os

from database.geometry import InvertSe3
from database.sql_table_loading import (
    load_extracted_targets_table,
    load_images_table,
    load_imu_data_table,
)
from database.types import SensorType

# TODO(Jack): Does it not make more sense to store the dictionary time keys as strings to prevent any problems with
#  dash/json serialization?


def process_images_table(table):
    if table is None:
        return None

    data = {}
    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            data[sensor_name] = {
                "type": SensorType.Camera,
                "measurements": {"images": {}},
            }

        timestamp_ns = int(row["timestamp_ns"])
        data[sensor_name]["measurements"]["images"][timestamp_ns] = row["data"]

    return data


# NOTE(Jack): Although technically the extracted targets are a calculated output of the calibration process frontend,
# for the purpose of the calibration process we treat them as a measurement too.
def process_extracted_targets_table(table, data):
    if table is None:
        return None

    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            raise KeyError(
                f"Error while loading data for {sensor_name} - sensor does not already exist."
            )

        timestamp_ns = int(row["timestamp_ns"])
        if timestamp_ns not in data[sensor_name]["measurements"]["images"]:
            raise KeyError(
                f"Error while loading data for {sensor_name} at time {timestamp_ns} - a corresponding image for the target was not found."
            )

        if "targets" not in data[sensor_name]["measurements"]:
            data[sensor_name]["measurements"].update({"targets": {}})

        target = row["data"]
        data[sensor_name]["measurements"]["targets"][timestamp_ns] = {
            "pixels": target["pixels"],
            "points": target["points"],
            "indices": target["indices"],
        }


# NOTE(Jack): When we start the project we enforced the foreign key relationship that a pose could only exist if there
# was a corresponding target. However once we got to spline calibration where the poses are interpolated, this
# restriction had to be removed. Therefore the only "foreign key" like relationship we check here is that the sensor has
# to exist.
#
# Of course, we could load the calibration_steps table and check that the poses step_name is valid, but for now we
# ignore that.
def process_poses_table(table, data):
    if table is None:
        return None

    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            raise KeyError(
                f"Error while loading data for {sensor_name} - sensor does not already exist."
            )

        if "poses" not in data[sensor_name]:
            data[sensor_name]["poses"] = {}

        step_name = row["step_name"]
        if step_name not in data[sensor_name]["poses"]:
            data[sensor_name]["poses"][step_name] = {}

        timestamp_ns = int(row["timestamp_ns"])

        # TODO(Jack): Make this something the user can configure in the frontend! Not here in the data loading.
        # NOTE(Jack): This logic here plays a surprisingly critical role in the broader scheme of things, so I want to
        # call attention to it. As mentioned elsewhere in the library, all the algorithms for camera calibration use
        # tf_co_w, as this is the tf which takes the target points in the world coordinate frame and puts them in the
        # camera optical frame. However, that transform is not suitable for world frame reference visualizations. In
        # order to provide the user a view of the calibration process that they understand from their world referenced
        # perspective we need to invert this tf to tf_w_co. That is what we are doing here.
        #
        # This means that the database holds the poses in a frame convention that the algorithm needs, but in order to
        # visualize them they need to be processed as we do here.
        pose_co_w = row.iloc[-6:].tolist()
        pose_w_co = InvertSe3(pose_co_w)
        data[sensor_name]["poses"][step_name][timestamp_ns] = pose_w_co

    return data


def process_reprojection_error_table(table, data):
    if table is None:
        return None

    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            raise KeyError(
                f"Error while loading data for {sensor_name} - sensor does not already exist."
            )

        # Enforce the foreign key relationship that in order for a reprojection error to exist we must already have a
        # matching target and pose loaded. Technically this is already enforced in the database, so maybe we should not
        # check it again here, but this is more for us to make sure we load the data properly.
        timestamp_ns = int(row["timestamp_ns"])
        step_name = row["step_name"]
        if (
            timestamp_ns not in data[sensor_name]["measurements"]["images"]
            or timestamp_ns not in data[sensor_name]["poses"][step_name]
        ):
            raise KeyError(
                f"Error while loading data for {sensor_name} in step {step_name} at time {timestamp_ns} - a corresponding target and/or pose was not found."
            )

        if "reprojection_error" not in data[sensor_name]:
            data[sensor_name]["reprojection_error"] = {}

        if step_name not in data[sensor_name]["reprojection_error"]:
            data[sensor_name]["reprojection_error"][step_name] = {}

        data[sensor_name]["reprojection_error"][step_name][timestamp_ns] = row["data"]


# NOTE(Jack): The imu data only consists of one length six array so we store it timestamped directly under the
# 'measurements' key.
def process_imu_data_table(table):
    if table is None:
        return None

    data = {}
    for index, row in table.iterrows():
        sensor_name = row["sensor_name"]
        if sensor_name not in data:
            data[sensor_name] = {"type": SensorType.Imu, "measurements": {}}

        timestamp_ns = int(row["timestamp_ns"])
        data[sensor_name]["measurements"][timestamp_ns] = row.iloc[-6:].tolist()

    return data


def load_data(db_path):
    if db_path is None or not os.path.isfile(db_path):
        return None

    data = {}

    table = load_images_table(db_path)
    if table is not None:
        data.update(process_images_table(table))

    table = load_extracted_targets_table(db_path)
    if table is not None:
        process_extracted_targets_table(table, data)

    table = load_imu_data_table(db_path)
    if table is not None:
        data.update(process_imu_data_table(table))

    return data
