from generated.extracted_target_pb2 import ArrayX2dProto
import sqlite3
import numpy as np
import pandas as pd
import os

from database.sql_statement_loading import load_sql


def build_arrayx2d(msg_data):
    rows = msg_data.rows
    data = np.array(msg_data.array_data, dtype=np.float64)

    if rows == 0 or data.size == 0:
        return np.empty((0, 2))
    else:
        # NOTE(Jack): We need the transpose here because eigen stores the data column wise. At least I think that is
        # why :)
        return data.reshape(2, rows).transpose()


def parse_proto(blob):
    msg = ArrayX2dProto()
    msg.ParseFromString(blob)

    return build_arrayx2d(msg).tolist()


# TODO(Jack): This is extremely similar to load_extracted_targets_df(), can we eliminate copy and paste at all?
def load_reprojection_error_df(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            df = pd.read_sql(
                load_sql('reprojection_error_select_all.sql'), conn
            )

            if 'data' not in df.columns:
                raise KeyError("'data' column not found in query result")

            def safe_parse(blob):
                try:
                    return parse_proto(blob)
                except Exception as e:
                    print(f"Failed to parse blob: {e}")
                    return None

            df['data'] = df['data'].apply(safe_parse)
    except Exception as e:
        print(f"Unexpected error in load_reprojection_error_df(db_path={db_path}): {e}")
        return None

    return df


# TODO(Jack): Lots of copy paste from the extracted target loading!
# TODO(Jack): Confirm naming and nesting makes logical sense.
def split_reprojection_error_by_sensor(df):
    targets_by_sensor = {}
    for sensor_name, group in df.groupby("sensor_name", sort=True):
        group = group.sort_values("timestamp_ns")

        frames = []
        for _, (_, row) in enumerate(group.iterrows()):
            # TODO(Jack): How should we handle dealing with both initial/optimized/etc. reprojection errors? Is having
            # here inside the dict acceptable?
            frames.append({
                "timestamp_ns": int(row["timestamp_ns"]),
                'type': row['type'],
                "data": row["data"],
            })

        targets_by_sensor[sensor_name] = frames

    return targets_by_sensor
