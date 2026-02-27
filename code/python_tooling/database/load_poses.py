import os
import sqlite3

import pandas as pd

from database.geometry import InvertSe3
from database.sql_statement_loading import load_sql
from database.types import PoseType


def add_camera_poses_df_to_camera_calibration_data(df, data):
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

        pose_type = row["type"]
        if pose_type not in PoseType:
            raise RuntimeError(
                f"Pose type {pose_type} not valid.",
            )



        if "poses" not in data[sensor]["frames"][timestamp]:
            data[sensor]["frames"][timestamp]["poses"] = {}
        data[sensor]["frames"][timestamp]["poses"][pose_type] = pose_w_co

    return data
