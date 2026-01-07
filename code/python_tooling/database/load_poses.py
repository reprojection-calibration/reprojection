import os
import pandas as pd
import sqlite3

from database.sql_statement_loading import load_sql


# TODO(Jack): Can we use an enum for table and type?
def load_poses_df(db_path, table, type):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            df = None
            if table == "camera":
                df = pd.read_sql(
                    load_sql('camera_poses_select.sql'), conn, params=(type,)
                )
    except Exception as e:
        print(f"Unexpected error in load_poses_df(db_path={db_path}, {table}, {type}): {e}")
        return None

    return df


def load_calibration_poses(db_path):
    df_initial_camera_poses = load_poses_df(db_path, 'camera', 'initial')
    if df_initial_camera_poses is not None:
        df_initial_camera_poses = df_initial_camera_poses.sort_values("timestamp_ns")

    df_optimized_camera_poses = load_poses_df(db_path, 'camera', 'optimized')
    if df_optimized_camera_poses is not None:
        df_optimized_camera_poses = df_optimized_camera_poses.sort_values("timestamp_ns")

    return df_initial_camera_poses, df_optimized_camera_poses
