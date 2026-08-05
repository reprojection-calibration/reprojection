#!/bin/bash

set -eoux pipefail

# NOTE(Jack): Sqlite has a weird complicated build process (at least compared to simple cmake workflows). Therefore we
# need to download this special preprocessed file (amalgamation?). Plead read the official docs to understand this if
# you are.

# NOTE(Jack): Version 3.45.1 was the version by default on my ubuntu 24 version when I checked (05.08.2026) and supports
# the syntax we require (eg. the 'RETURNING id' syntax)
file_name="sqlite-autoconf-3450100.tar.gz"
wget "https://www.sqlite.org/2024/${file_name}"

dir="/buildroot/sqlite"
mkdir --parents "${dir}"

tar \
    --extract \
    --gzip \
    --file "${file_name}" \
    --directory "${dir}" \
    --strip-components=1

cd "${dir}"

# NOTE(Jack): Got some linking error with gdal during the library compilation so we added this flag.
CFLAGS="-DSQLITE_ENABLE_COLUMN_METADATA" \
  ./configure --prefix "/opt/reprojection"
make --jobs="$(nproc)"
make install