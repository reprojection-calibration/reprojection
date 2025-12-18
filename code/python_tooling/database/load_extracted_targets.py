from database.extracted_target_pb2 import ExtractedTargetProto
import sqlite3
import numpy as np

from database.sql_statement_loading import load_sql
from pyasn1.compat.octets import null


class ExtractedTarget:
    def __init__(self, pixels, points, indices):
        assert pixels.shape[0] == points.shape[0] == indices.shape[0]

        self.pixels = pixels
        self.points = points
        self.indices = indices


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

    targets = {}
    for ts, sensor, blob in cur.fetchall():
        msg = ExtractedTargetProto()
        msg.ParseFromString(blob)

        data = ExtractedTarget(
            build_pixels(msg.bundle), build_points(msg.bundle), build_indices(msg)
        )

        # TODO(Jack): What is the deal here with this set default thing?
        targets.setdefault(sensor, {})[ts] = data

    conn.close()

    return targets
