import sqlite3
from collections import defaultdict

from database.sql_statement_loading import load_sql


# TODO(Jack): Can we use an enum for table and type?
def load_poses(db_path, table, type):
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    # TODO(Jack): Is there a better way to parameterize the table type?
    # TODO(Jack): For some reason I could not pass the sql statement as text variable to cur.execute() but instead
    # had to copy and paste the entire logic twice.
    if table == "camera":
        cur.execute(
            load_sql("camera_poses_select.sql"), (type,)
        )
    elif table == "external":
        cur.execute(
            load_sql("external_poses_select.sql"), (type,)
        )

    rows = cur.fetchall()
    conn.close()

    poses_dict = defaultdict(dict)
    for ts, sensor, *rest in rows:
        poses_dict[sensor][ts] = list(rest)

    return poses_dict
