from generated.extracted_target_pb2 import ExtractedTargetProto
import sqlite3
import numpy as np
import pandas as pd
import os

from database.sql_statement_loading import load_sql


def build_pixels(msg_data):
    rows = msg_data.pixel_rows
    data = np.array(msg_data.pixel_data, dtype=np.float64)

    if rows == 0 or data.size == 0:
        return np.empty((0, 2))
    else:
        # NOTE(Jack): We need the transpose here because eigen stores the data column wise. At least I think that is
        # why :)
        return data.reshape(2, rows).transpose()


def build_points(msg_data):
    rows = msg_data.point_rows
    data = np.array(msg_data.point_data, dtype=np.float64)

    if rows == 0 or data.size == 0:
        return np.empty((0, 3))
    else:
        return data.reshape(3, rows).transpose()


def build_indices(msg_data):
    rows = msg_data.indices_rows
    data = np.array(msg_data.indices_data, dtype=np.int32)

    if rows == 0 or data.size == 0:
        return np.empty((0, 2))
    else:
        return data.reshape(2, rows).transpose()


# TODO(Jack): I really am not sure here, but is storing a dict in a pandas datatable a crime? Might be but for not the
# clarity of the abstraction takes precedence! But lets be ready to refactor this if it turns into a problem.
def parse_proto(blob: bytes) -> ExtractedTargetProto:
    msg = ExtractedTargetProto()
    msg.ParseFromString(blob)

    return {'pixels': build_pixels(msg.bundle).tolist(),
            'points': build_points(msg.bundle).tolist(),
            'indices': build_indices(msg).tolist()}


def load_all_extracted_targets(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            sql_query = load_sql('extracted_targets_select_all.sql')
            df = pd.read_sql(sql_query, conn)

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
        print(f"Unexpected error in load_all_extracted_targets(db_path={db_path}): {e}")
        return None

    return df
