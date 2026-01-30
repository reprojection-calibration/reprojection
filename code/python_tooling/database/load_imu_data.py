import os
import sqlite3

import pandas as pd

from database.sql_statement_loading import load_sql


def load_imu_data_df(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            df = pd.read_sql(load_sql("imu_data_select_all.sql"), conn)
    except Exception as e:
        print(f"Unexpected error in load_imu_data_df(db_path={db_path}): {e}")
        return None

    return df
