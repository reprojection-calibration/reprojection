#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh

download_and_extract \
    http://ceres-solver.org/ceres-solver-2.2.0.tar.gz \
    /buildroot

cmake_build_install \
    /buildroot/ceres-solver-2.2.0 \
    /buildroot/ceres-"${CMAKE_BUILD_TYPE}" \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF


# Install april tags from git repository
clone_repo \
    https://github.com/AprilRobotics/apriltag.git \
    v3.4.5 \
    /buildroot/apriltag

cmake_build_install \
    /buildroot/apriltag \
    /buildroot/apriltag-"${CMAKE_BUILD_TYPE}"


# Install tomlplusplus from git repository
clone_repo \
    https://github.com/marzer/tomlplusplus.git \
    v3.4.0 \
    /buildroot/tomlplusplus

cmake_build_install \
    /buildroot/tomlplusplus \
    /buildroot/tomlplusplus-"${CMAKE_BUILD_TYPE}"