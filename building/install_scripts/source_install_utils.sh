#!/bin/bash

set -eoux pipefail

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
    local source_dir="${1}"
    shift 1

    local source_name
    source_name="$(basename "${source_dir}")"

    local build_dir="/buildroot/build-${source_name}"

    # NOTE(Jack): Later -D arguments override earlier ones!
    cmake -B "${build_dir}" -S "${source_dir}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_TESTING=OFF \
        -GNinja \
        "$@"

    cmake --build "${build_dir}"
    cmake --install "${build_dir}"
}

cmake_test() {
    local build_dir="$1"

    cd "${build_dir}"
    ctest --output-on-failure --progress
}