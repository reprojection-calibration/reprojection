#!/bin/bash

set -eoux pipefail

CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Release}

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
    local buildroot=$1
    local name=$2
    shift 2

    local source_dir="${buildroot}/${name}"
    local build_dir="${source_dir}-${CMAKE_BUILD_TYPE}"

    cmake -S "${source_dir}" -B "${build_dir}" \
        -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}" \
        -DCMAKE_INSTALL_PREFIX="/opt/${name}" \
        -GNinja \
        "$@"

    cmake --build "${build_dir}"
    cmake --install "${build_dir}"
}