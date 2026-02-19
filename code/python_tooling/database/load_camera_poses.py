import os
import sqlite3

import pandas as pd

from database.sql_statement_loading import load_sql
from database.types import PoseType


def load_camera_poses_df(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            df = pd.read_sql(load_sql("camera_poses_select_all.sql"), conn)
    except Exception as e:
        print(f"Unexpected error in load_camera_poses_df(db_path={db_path}): {e}")
        return None

    return df


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

        pose = row.iloc[-6:].tolist()

        if "poses" not in data[sensor]["frames"][timestamp]:
            data[sensor]["frames"][timestamp]["poses"] = {}
        data[sensor]["frames"][timestamp]["poses"][pose_type] = pose

    return data
