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


# NOTE(Jack): The images hold a special place in the database that sets it apart from all tables. And that is, it is the
# only table with NO foreign key relationship. That means that it defines what can and what can not exist as data in the
# rest of the database. All other tables have one way or another a foreign key relationship on the image table.
def image_df_to_camera_calibration_data(df):
    data = {}
    for index, row in df.iterrows():
        sensor = row['sensor_name']
        timestamp = row['timestamp_ns']

        if sensor not in data:
            data[sensor] = {'frames': {}}

        data[sensor]['frames'][timestamp] = {'image': row['data']}

    return data
