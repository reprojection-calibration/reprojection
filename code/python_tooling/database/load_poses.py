import sqlite3
import numpy as np


# TODO(Jack): Paramaterize the table name
# TODO(Jack): Can we use an enum for table and type?
def load_poses(db_path, table, type):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    # TODO(Jack): Is there a better way to paramaterize the table type?
    # ERROR(Jack): Dangerous copy paste, use common sql defs for all code
    # TODO(Jack): For some reason I could not pass the sql statement as text variable to cur.execute() but instead
    # had to copy and paste the entire logic twice.
    if table == "camera":
        cur.execute("SELECT timestamp_ns, sensor_name, type, rx, ry, rz, x, y, z "
                    "FROM camera_poses WHERE type = ? ORDER BY timestamp_ns ASC", (type,))
    elif table == "external":
        cur.execute("SELECT timestamp_ns, sensor_name, type, rx, ry, rz, x, y, z "
                    "FROM external_poses WHERE type = ? ORDER BY timestamp_ns ASC", (type,))

    rows = cur.fetchall()
    conn.close()

    return rows
