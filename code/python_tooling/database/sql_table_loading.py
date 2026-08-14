import logging
import os
import sqlite3

import pandas as pd

from database.proto_parsing import parse_array_x2d_proto, parse_extracted_target_proto
from database.sql_statement_loading import load_sql

log = logging.getLogger("reprojection")


def load_table(db_path, sql_query_file):
    if not os.path.isfile(db_path):
        raise FileNotFoundError(f"Database file does not exist: {db_path}")

    sql_query = load_sql(sql_query_file)
    try:
        with sqlite3.connect(db_path) as conn:
            table = pd.read_sql(sql_query, conn)
    except Exception as err:
        log.warning(
            f"Failed to load SQL table. db_path='{db_path}', query_file='{sql_query_file}', error='{err}'"
        )
        return None

    return table


def load_table_blob(db_path, sql_query_file, parser, blob_column_id="data"):
    table = load_table(db_path, sql_query_file)
    if table is None:
        return None
    elif blob_column_id not in table.columns:
        raise KeyError("Column '{}' not found in query result.", blob_column_id)

    table["data"] = table["data"].apply(parser)

    return table


def load_calibration_database(db_path):
    db = {}

    # Tables that do not require blob parsing.
    for table_name in (
        "assets",
        "camera_info",
        "camera_poses",
        "extrinsics",
        "images_timestamps",
        "imu_data",
        "imu_errors",
        "intrinsics",
        "target_info",
        "workflow_assets",
        "workflow_steps",
        "workflows",
    ):
        if (table := load_table(db_path, table_name + "_select_all.sql")) is not None:
            db[table_name] = table

    # Tables that do require blob parsing.
    blob_tables = {
        "extracted_targets": parse_extracted_target_proto,
        "reprojection_errors": parse_array_x2d_proto,
    }
    for table_name, parser in blob_tables.items():
        table = load_table_blob(db_path, f"{table_name}_select_all.sql", parser)
        if table is not None:
            db[table_name] = table

    # Explicitly keep the IDs as columns in the "single row" tables.
    for table_name in (
        "camera_info",
        "target_info",
    ):
        if table_name not in db:
            continue

        db[table_name] = db[table_name].set_index(["step_id", "asset_id"], drop=False)

    # For the "multi row" tables the ID is automatically preserved in the table even when using a multindex like we do
    # here. If we also set drop=False here then it would be repeated twice which would not be so nice - that is why we
    # treat the single row and multi row tables separately.
    for table_name in (
        "camera_poses",
        "extracted_targets",
        "images_timestamps",
        "imu_data",
        "imu_errors",
        "intrinsics",
        "reprojection_errors",
    ):
        if table_name not in db:
            continue

        db[table_name] = db[table_name].set_index(["step_id", "asset_id"])

    # WARN(Jack): The extrinsics table is unique because it has two assets not just a single one. We need to figure a
    # canonical robust way to handle this.
    if "exintrinsics" in db:
        db["extrinsics"] = db["extrinsics"].set_index(
            ["step_id"],
            drop=False,
        )

    return db
