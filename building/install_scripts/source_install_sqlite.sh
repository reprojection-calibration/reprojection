#!/bin/bash

set -eoux pipefail

# We need to source install sqlite because the apt available on ubuntu 20.04 (used by the ROS1 app) does not support the
# "RETURNING id;" syntax we use a lot.

source /temporary/building/install_scripts/source_install_utils.sh

# NOTE(Jack): Sqlite has a weird complicated build process (at least compared to simple cmake workflows). Therefore we
# need to download this special preprocessed file (amalgamation?). Plead read the official docs to understand this if
# you are.
# NOTE(Jack): Version 3.45.1 was the version by default on my ubuntu 24 version when I checked (05.08.2026) and supports
# the syntax we require (eg. the 'RETURNING id' syntax).

buildroot="/buildroot"

name="sqlite-autoconf-3450100"
download_and_extract \
    "https://www.sqlite.org/2024/${name}.tar.gz" \
    "${buildroot}"

cd "${buildroot}/${name}"

# NOTE(Jack): Got some linking error with gdal during the library compilation so we added this flag.
CFLAGS="-DSQLITE_ENABLE_COLUMN_METADATA" \
  ./configure --prefix "/opt/reprojection"
make --jobs="$(nproc)"
make install