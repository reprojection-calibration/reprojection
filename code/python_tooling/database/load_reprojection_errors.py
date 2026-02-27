import os
import sqlite3

import numpy as np
import pandas as pd

from database.sql_statement_loading import load_sql
from database.types import PoseType
from generated.extracted_target_pb2 import ArrayX2dProto


# TODO(Jack): All of these functions are pretty similar to eachother, is there any way that we can combine them and
#  eliminate code duplication?
def add_reprojection_errors_df_to_camera_calibration_data(df, data):
    for index, row in df.iterrows():
        sensor = row["sensor_name"]
        if sensor not in data:
            raise RuntimeError(
                f"Sensor {sensor} not present in camera calibration data dictionary",
            )

        timestamp = int(row["timestamp_ns"])
        if timestamp not in data[sensor]["frames"]:
            raise RuntimeError(
                f"Timestamp {timestamp} for sensor {sensor} not present in camera calibration data dictionary"
            )

        # TODO(Jack): The naming here no longer makes sense! The enum PoseType is too specific, what we really want is
        #  something which describes if a value is calculated before or after optimization
        reprojection_error_type = row["type"]
        if reprojection_error_type not in PoseType:
            raise RuntimeError(
                f"Pose type {reprojection_error_type} not valid.",
            )

        reprojection_error = row["data"]

        if "reprojection_errors" not in data[sensor]["frames"][timestamp]:
            data[sensor]["frames"][timestamp]["reprojection_errors"] = {}
        data[sensor]["frames"][timestamp]["reprojection_errors"][
            reprojection_error_type
        ] = reprojection_error

    return data
