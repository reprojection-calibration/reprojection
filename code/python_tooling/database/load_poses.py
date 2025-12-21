import sqlite3
from collections import defaultdict

from database.sql_statement_loading import load_sql
import pandas as pd


# TODO(Jack): Can we use an enum for table and type?
def load_poses(db_path, table, type):
    conn = sqlite3.connect(db_path)

    # TODO(Jack): Is there a better way to parameterize the table type?
    # TODO(Jack): For some reason I could not pass the sql statement as text variable to cur.execute() but instead
    # had to copy and paste the entire logic twice.
    df = None
    if table == "camera":
        df = pd.read_sql(load_sql('camera_poses_select.sql'), conn, params=(type,))
        print(df)
    elif table == "external":
        df = pd.read_sql(load_sql('external_poses_select.sql'), conn, params=(type,))

    conn.close()

    return df
