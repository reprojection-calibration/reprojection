import sqlite3

# TODO(Jack): Setup a environment so we do not need to deal with issues like this!
try:
    from importlib.resources import files  # Python ≥3.9
except ImportError:
    from importlib_resources import files  # Python <3.9


def load_sql(name: str) -> str:
    return files("database.sql").joinpath(name).read_text()


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
        cur.execute(load_sql("camera_poses_select.sql"), (type,))
    elif table == "external":
        cur.execute(load_sql("external_poses_select.sql"), (type,))

    rows = cur.fetchall()
    conn.close()

    return rows
