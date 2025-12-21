import sqlite3
from database.sql_statement_loading import load_sql
import pandas as pd


# TODO(Jack): Can we use an enum for table and type?
def load_poses(db_path, table, type):
    conn = sqlite3.connect(db_path)

    df = None
    if table == "camera":
        df = pd.read_sql(load_sql('camera_poses_select.sql'), conn, params=(type,))
    elif table == "external":
        df = pd.read_sql(load_sql('external_poses_select.sql'), conn, params=(type,))

    conn.close()

    return df
