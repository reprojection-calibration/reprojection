#!/bin/bash

set -eoux pipefail

CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Release}
BUILD_TESTING=${BUILD_TESTING:-OFF}

clone_repo() {
    local repo=$1
    local branch=$2
    local dest=$3

    git clone --branch "$branch" --depth 1 "$repo" "$dest"
}

download_and_extract() {
    local url=$1
    local dest=$2

    wget --directory-prefix="$dest" "$url"
    tar -xf "$dest/$(basename "$url")" -C "$dest"
}

cmake_build_install() {
    local build_dir=$1
    local source_dir=$2
    shift 2

    cmake -B "${build_dir}" -S "${source_dir}" \
        -DBUILD_TESTING="${BUILD_TESTING}" \
        -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
        -DCMAKE_INSTALL_PREFIX="/opt/reprojection" \
        -GNinja \
        "$@"

    cmake --build "${build_dir}"

    if [[ "${BUILD_TESTING}" == "ON" ]]; then
      # NOTE(Jack): The --test-dir option is not supported under cmake version 3.20 - ubuntu 20 defaults to 3.16 - so we just
      # do it manually here.
      cd "${build_dir}"
      ctest --output-on-failure --progress
    fi

    cmake --install "${build_dir}"
}