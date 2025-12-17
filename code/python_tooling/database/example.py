from database.extracted_target_pb2 import ExtractedTargetProto
import sqlite3
import numpy as np


def add_one(number):
    return number + 1


def load_extracted_targets(db_path):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()
    # ERROR(Jack): Dangerous copy paste, see TODO below.
    # TODO(Jack): Load this sql defintiion from the sql folder!
    cur.execute(
        "SELECT timestamp_ns, sensor_name, data "
        "FROM extracted_targets ORDER BY timestamp_ns ASC"
    )

    targets = {}
    for ts, sensor, blob in cur.fetchall():
        msg = ExtractedTargetProto()
        msg.ParseFromString(blob)

        rows = msg.bundle.point_rows
        data = np.array(msg.bundle.point_data, dtype=np.float64)

        if rows == 0 or data.size == 0:
            points = np.empty((0, 3))
        else:
            points = data.reshape(3, rows).transpose()  # Eigen::MatrixX3d

        targets.setdefault(sensor, {})[ts] = points

    conn.close()

    return targets
