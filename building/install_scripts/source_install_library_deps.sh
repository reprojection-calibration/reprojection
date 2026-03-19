#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh


name=apriltag
clone_repo \
    https://github.com/AprilRobotics/${name}.git \
    v3.4.5 \
    /buildroot/${name}
cmake_build_install \
    /buildroot/${name} \
    /buildroot/${name}-"${CMAKE_BUILD_TYPE}"

name=eigen
clone_repo \
    https://github.com/eigen-mirror/${name}.git \
    3.4.0 \
    /buildroot/${name}
cmake_build_install \
    /buildroot/${name} \
    /buildroot/${name}-"${CMAKE_BUILD_TYPE}"

name=ceres-solver-2.2.0
download_and_extract \
    http://ceres-solver.org/${name}.tar.gz \
    /buildroot
cmake_build_install \
    /buildroot/${name} \
    /buildroot/ceres-"${CMAKE_BUILD_TYPE}" \
    -DBUILD_BENCHMARKS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTING=OFF

name=tomlplusplus
clone_repo \
    https://github.com/marzer/${name}.git \
    v3.4.0 \
    /buildroot/${name}
cmake_build_install \
    /buildroot/${name} \
    /buildroot/${name}-"${CMAKE_BUILD_TYPE}"