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
ceres_sha256="12efacfadbfdc1bbfa203c236e96f4d3c210bed96994288b3ff0c8e7c6f350d4"
download_and_extract \
    https://github.com/ceres-solver/ceres-solver/archive/refs/tags/2.2.0.tar.gz \
    "${buildroot}/${name}" \
    "${ceres_sha256}" \
    1
cmake_build \
    "${buildroot}/${name}" \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_EXAMPLES=OFF
cmake_install \
    "${buildroot}/${name}"