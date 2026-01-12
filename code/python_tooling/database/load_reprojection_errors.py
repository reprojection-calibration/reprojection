from generated.extracted_target_pb2 import ArrayX2dProto
import sqlite3
import numpy as np
import pandas as pd
import os

from database.sql_statement_loading import load_sql


# TODO(Jack): Due to undesirable dual definition of pixels with the extracted target and the array which stores the
#  reprojecion error we need essentially duplicated this function in two places. One day when we unify the
#  representation we can combine them and use one common function.
def build_array_x2d(msg_data):
    rows = msg_data.rows
    data = np.array(msg_data.data_array, dtype=np.float64)

    if rows == 0 or data.size == 0:
        return np.empty((0, 2))
    else:
        return data.reshape(2, rows).transpose()


def parse_proto(blob):
    msg = ArrayX2dProto()
    msg.ParseFromString(blob)

    return build_array_x2d(msg).tolist()


# TODO(Jack): This function is very similar to load_extracted_targets_df(), can we eliminate copy and paste? One big
# difference is that the extracted targets do not have the "type" identifier in their table.
def load_reprojection_errors_df(db_path):
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
        print(f"Unexpected error in load_reprojection_errors_df(db_path={db_path}): {e}")
        return None

    return df
