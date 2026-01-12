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
            df = pd.read_sql(
                load_sql('camera_poses_select_all.sql'), conn
            )
    except Exception as e:
        print(f"Unexpected error in load_camera_poses_df(db_path={db_path}): {e}")
        return None

    return df
