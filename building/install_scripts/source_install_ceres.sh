#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh

buildroot="/buildroot"

name="eigen"
clone_repo \
    https://github.com/eigen-mirror/${name}.git \
    3.4.0 \
    "${buildroot}/${name}"
cmake_build \
    "${buildroot}/${name}"
cmake_install \
    "${buildroot}/${name}"

name="ceres-solver-2.2.0"
download_and_extract \
    http://ceres-solver.org/${name}.tar.gz \
    "${buildroot}"
cmake_build \
    "${buildroot}/${name}" \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_EXAMPLES=OFF
cmake_install \
    "${buildroot}/${name}"