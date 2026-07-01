import os
from pathlib import Path


def sql_root():
    value = os.environ.get("REPROJECTION_SQL_DIR")
    if value is None:
        raise RuntimeError("REPROJECTION_SQL_DIR is not set")

    return Path(value)


# NOTE(Jack): `code/sql/` is symlinked to the python database package folder `sql/`. This means we have a single
# source of truth. But we run the risk that the symlink causes us trouble one day! Let's see what happens...
#
# Turns out it did cause a lot of problems! For the docker workflow we need to COPY the .sql files directly into
# the `python_tooling/database` package. See the comment there for more details.
def load_sql(name):
    return (sql_root() / name).read_text(encoding="utf-8")
