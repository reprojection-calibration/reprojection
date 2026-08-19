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

# NOTE(Jack): We want control over when we build/install/test so we have three different functions. The only annoying
# thing is then keeping track of the build directory. Therefore we have this helper here which given the source
# directory will always return a directory in /buildroot named the same way. A little round abouty but it gets the job
# done!
cmake_build_dir() {
    local source_dir="${1}"
    local source_name
    source_name="$(basename "${source_dir}")"

    # NOTE(Jack): That this is the only way to return a value from a bash function is insane...
    printf '/buildroot/build-%s\n' "${source_name}"
}

cmake_build() {
    local source_dir="${1}"
    shift

    local build_dir
    build_dir="$(cmake_build_dir "${source_dir}")"

    # NOTE(Jack): Later -D arguments override earlier ones!
    cmake -B "${build_dir}" -S "${source_dir}" \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_TESTING=OFF \
        -GNinja \
        "$@"

    cmake --build "${build_dir}"
}

cmake_install() {
    local source_dir="${1}"
    shift

    local build_dir
    build_dir="$(cmake_build_dir "${source_dir}")"

    cmake --install "${build_dir}"
}

cmake_test() {
    local source_dir="${1}"
    shift

    local build_dir
    build_dir="$(cmake_build_dir "${source_dir}")"

    cd "${build_dir}"
    ctest --output-on-failure --progress
}