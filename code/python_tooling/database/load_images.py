import os
import pandas as pd
import sqlite3

from database.sql_statement_loading import load_sql


def load_images_df(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            df = pd.read_sql(
                load_sql('images_select_all.sql'), conn
            )
    except Exception as e:
        print(f"Unexpected error in load_poses(db_path={db_path}): {e}")
        return None

    return df


def split_images_by_sensor(df):
    images_by_sensor = {}
    for sensor_name, group in df.groupby("sensor_name", sort=True):
        group = group.sort_values("timestamp_ns")

        frames = []
        for _, (_, row) in enumerate(group.iterrows()):
            frames.append({
                "timestamp_ns": int(row["timestamp_ns"]),
                "data": row["data"]
            })

        images_by_sensor[sensor_name] = frames

    return images_by_sensor
