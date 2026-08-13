import os
from pathlib import Path


# NOTE(Jack): Defining this as a global is hacky, but ensuring that we have the same exact copy of the SQL statements
# used by both the library and the python tooling when doing local development is not trivial. If someone has a better
# way to solve this please let me know!
def sql_root():
    value = os.environ.get("REPROJECTION_SQL_PYTHON_DIR")
    if value is None:
        raise RuntimeError("REPROJECTION_SQL_PYTHON_DIR is not set!")

    return Path(value)


def load_sql(name):
    return (sql_root() / name).read_text(encoding="utf-8")
