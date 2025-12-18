# TODO(Jack): Setup a environment so we do not need to deal with issues like this for local and docker dev!
try:
    from importlib.resources import files
except ImportError:
    from importlib_resources import files


def load_sql(name: str) -> str:
    # NOTE(Jack): `code/sql/` is symlinked to the python database package folder `sql/`. This means we have a single
    # source of truth. But we run the risk that the symlink causes us trouble one day! Let' see what happens...
    return files("database.sql").joinpath(name).read_text()
