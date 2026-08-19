#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh

build_type=Release
buildroot="/buildroot"


name="apriltag"
clone_repo \
    https://github.com/AprilRobotics/${name}.git \
    v3.4.5 \
    "${buildroot}/${name}"
cmake_build \
    "${buildroot}/${name}" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PYTHON_WRAPPER=OFF
cmake_install \
    "${buildroot}/${name}"

name="spdlog"
clone_repo \
    https://github.com/gabime/${name}.git \
    v1.17.0 \
    "${buildroot}/${name}"
cmake_build \
    "${buildroot}/${name}" \
    -DBUILD_SHARED_LIBS=ON
cmake_install \
    "${buildroot}/${name}"

name="tomlplusplus"
clone_repo \
    https://github.com/marzer/${name}.git \
    v3.4.0 \
    "${buildroot}/${name}"
cmake_build \
    "${buildroot}/${name}"
cmake_install \
    "${buildroot}/${name}"