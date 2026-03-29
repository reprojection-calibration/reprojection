#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh

name=spdlog
clone_repo \
    https://github.com/gabime/${name}.git \
    v1.17.0 \
    /buildroot/${name}
cmake_build_install \
    /buildroot/${name} \
    /buildroot/${name}-"${CMAKE_BUILD_TYPE}" \
    -DBUILD_SHARED_LIBS=ON