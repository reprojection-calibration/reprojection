import os
import sqlite3

import pandas as pd

from database.proto_parsing import parse_array_x2d_proto, parse_extracted_target_proto
from database.sql_statement_loading import load_sql


# TODO(Jack): Make generic loading function for the basic case and use that to eliminate copy and paste!
def load_images_table(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            table = pd.read_sql(load_sql("images_select_all.sql"), conn)
    except Exception as e:
        print(e)
        return None

    return table


# TODO(Jack): This function is very similar to load_reprojection_errors_table(), can we eliminate copy and paste? One big
#  difference is that the extracted targets do not have the "type" identifier in their table.
def load_extracted_targets_table(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            table = pd.read_sql(load_sql("extracted_targets_select_all.sql"), conn)

            if "data" not in table.columns:
                raise KeyError("'data' column not found in query result")

            def safe_parse(blob):
                try:
                    return parse_array_x2d_proto(blob)
                except Exception as e:
                    print(f"Failed to parse blob: {e}")
                    return None

            table["data"] = table["data"].apply(safe_parse)
    except Exception as e:
        print(e)
        return None

    return table


def load_poses_table(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            table = pd.read_sql(load_sql("poses_select_all.sql"), conn)
    except Exception as e:
        print(e)
        return None

    return table


def load_reprojection_errors_table(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            table = pd.read_sql(load_sql("reprojection_error_select_all.sql"), conn)

            if "data" not in table.columns:
                raise KeyError("'data' column not found in query result")

            def safe_parse(blob):
                try:
                    return parse_array_x2d_proto(blob)
                except Exception as e:
                    print(f"Failed to parse blob: {e}")
                    return None

            table["data"] = table["data"].apply(safe_parse)
    except Exception as e:
        print(e)
        return None

    return table


def load_imu_data_table(db_path):
    if not os.path.isfile(db_path):
        print(f"Database file does not exist: {db_path}")
        return None

    try:
        with sqlite3.connect(db_path) as conn:
            table = pd.read_sql(load_sql("imu_data_select_all.sql"), conn)
    except Exception as e:
        print(e)
        return None

    return table
