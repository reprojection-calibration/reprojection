import os
from pathlib import Path


def sql_root():
    value = os.environ.get("REPROJECTION_SQL_DIR")
    if value is None:
        raise RuntimeError("REPROJECTION_SQL_DIR is not set")

    return Path(value)


def load_sql(name):
    return (sql_root() / name).read_text(encoding="utf-8")
