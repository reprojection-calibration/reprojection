import os
import sqlite3

import pandas as pd

from database.sql_statement_loading import load_sql
from database.types import PoseType
from database.geometry import InvertSe3


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

        if "poses" not in data[sensor]["frames"][timestamp]:
            data[sensor]["frames"][timestamp]["poses"] = {}
        data[sensor]["frames"][timestamp]["poses"][pose_type] = pose_w_co

    return data
