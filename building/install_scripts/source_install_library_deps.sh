#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh


# TODO(Jack): Disable building opencv demo and examples
name=apriltag
clone_repo \
    https://github.com/AprilRobotics/${name}.git \
    v3.4.5 \
    /buildroot/${name}
cmake_build_install \
    /buildroot/${name} \
    /buildroot/${name}-"${CMAKE_BUILD_TYPE}"

# NOTE(Jack): The library requires eigen 3.4. On ubuntu 20 the default apt package version is eigen 3.3.7, therefore
# we decide to compile from source here for all versions, which also means we need to compile ceres from source (are we
# sure?)...
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