from generated.extracted_target_pb2 import ExtractedTargetProto
import sqlite3
import numpy as np
from collections import defaultdict

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


def load_all_extracted_targets(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    cur.execute(
        load_sql("extracted_targets_select_all.sql")
    )

    data = defaultdict(lambda: {"data": defaultdict(lambda: {"extracted_target": {}})})
    for ts, sensor, blob in cur.fetchall():
        msg = ExtractedTargetProto()
        msg.ParseFromString(blob)

        data[sensor]['data'][ts]["extracted_target"]["pixels"] = build_pixels(msg.bundle).tolist()
        data[sensor]['data'][ts]["extracted_target"]["points"] = build_points(msg.bundle).tolist()
        data[sensor]['data'][ts]["extracted_target"]["indices"] = build_indices(msg).tolist()

    conn.close()

    return data
