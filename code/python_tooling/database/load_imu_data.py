import os
import sqlite3

import pandas as pd

from database.sql_statement_loading import load_sql


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
