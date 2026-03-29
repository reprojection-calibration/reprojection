#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh

buildroot="/buildroot"

name="eigen"
clone_repo \
    https://github.com/eigen-mirror/${name}.git \
    3.4.0 \
    "${buildroot}/${name}"
cmake_build_install \
    "${buildroot}" \
    "${name}"

rm -r /buildroot/eigen-Release

name="ceres-solver-2.2.0"
download_and_extract \
    http://ceres-solver.org/${name}.tar.gz \
    "${buildroot}"
cmake_build_install \
    "${buildroot}" \
    "${name}" \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF \
    -DCMAKE_PREFIX_PATH="/opt/reprojection/eigen"
