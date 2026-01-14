import os
import pandas as pd
import sqlite3

from database.sql_statement_loading import load_sql


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
        if sensor not in    data:
            raise RuntimeError(
                f"Sensor {sensor} not present in camera calibration data dictionary",
            )

        timestamp = row["timestamp_ns"]
        if timestamp not in data[sensor]["frames"]:
            raise RuntimeError(
                f"Timestamp {timestamp} for sensor {sensor} not present in camera calibration data dictionary"
            )

        # TODO(Jack): Should we formalize the loading of the pose type here to check that it is part of the PoseType
        #  enum? See similar logic in add_reprojection_errors_df_to_camera_calibration_data().
        pose_type = row["type"]
        pose = row.iloc[-6:].tolist()

        if "poses" not in data[sensor]["frames"][timestamp]:
            data[sensor]["frames"][timestamp]["poses"] = {}
        data[sensor]["frames"][timestamp]["poses"][pose_type] = pose

    return data
