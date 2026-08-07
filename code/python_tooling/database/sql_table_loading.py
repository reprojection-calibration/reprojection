import logging
import os
import sqlite3
import textwrap

import pandas as pd

from database.sql_statement_loading import load_sql

log = logging.getLogger("reprojection")


# Used for all tables whose columns can be loaded without blob deserialization.
def load_table(db_path, sql_query_file):
    if not os.path.isfile(db_path):
        raise FileNotFoundError(f"Database file does not exist: {db_path}")

    sql_query = load_sql(sql_query_file)
    try:
        with sqlite3.connect(db_path) as conn:
            table = pd.read_sql(sql_query, conn)
    except Exception as e:
        return None

    return table


def load_table_blob(db_path, sql_query_file, parser, blob_column_id='data'):
    table = load_table(db_path, sql_query_file)

    if table is None:
        return None
    elif blob_column_id not in table.columns:
        raise KeyError("Column '{}' not found in query result.", blob_column_id)

    table["data"] = table["data"].apply(parser)

    return table


def load_calibration_database(db_path):
    db = {}

    for table_name in ("assets_select_all", "workflow_assets_select_all", "workflow_steps_select_all"):
        if table := load_table(db_path, table_name) is not None:
            db[table_name] = table

    return db

    # from database.proto_parsing import parse_array_x2d_proto, parse_extracted_target_proto

    # def safe_parse(blob):
    #    try:
    #        return parse_extracted_target_proto(blob)
    #    except Exception as e:
    #        print(f"Failed to parse blob: {e}")
    #        return None

    # def safe_parse(blob):
    #    try:
    #        return parse_array_x2d_proto(blob)
    #    except Exception as e:
    #        print(f"Failed to parse blob: {e}")
    #        return None
