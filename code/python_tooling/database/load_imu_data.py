import os
import sqlite3

import pandas as pd

from database.sql_statement_loading import load_sql

# TODO(Jack): Right now we are just starting to build up the imu data pipeline quickly, therefore we are following many
#  of the same patterns laid down in the camera calibration data framework. However we need to keep our eyes peels for
#  places where the material difference between the two data spaces means that they are so different the implementation
#  will need to be materially different. For example the foreign key constraints for the imu calibration process are not
#  so strict which means there is more freedom to mess things up!


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


# TODO(Jack): Are we sure this should not edit the data dict by reference? Or it should really be created here?
def imu_data_df_to_imu_calibration_data(df):
    data = {}
    for index, row in df.iterrows():
        sensor = row["sensor_name"]
        timestamp = int(row["timestamp_ns"])
        measurement = row.iloc[-6:].tolist()

        if sensor not in data:
            data[sensor] = {"frames": {}}

        data[sensor]["frames"][timestamp] = {"imu_measurement": measurement}

    return data
