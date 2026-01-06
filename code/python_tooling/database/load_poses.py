import sqlite3
from database.sql_statement_loading import load_sql
import pandas as pd
import os


# TODO(Jack): Can we use an enum for table and type?
def load_poses(db_path, table, type):
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
            elif table == "external":
                df = pd.read_sql(
                    load_sql('external_poses_select.sql'), conn, params=(type,)
                )
    except Exception as e:
        print(f"Unexpected error in load_poses(db_path={db_path}, {table}, {type}): {e}")
        return None

    return df
