from importlib.resources import files


# NOTE(Jack): `code/sql/` is symlinked to the python database package folder `sql/`. This means we have a single
# source of truth. But we run the risk that the symlink causes us trouble one day! Let's see what happens...
#
# Turns out it did cause a lot of problems! For the docker workflow we need to COPY the .sql files directly into
# the `python_tooling/database` package. See the comment there for more details.
def load_sql(name):
    return files("database.sql").joinpath(name).read_text()
