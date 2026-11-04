#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh

buildroot="/buildroot"

name="feature-extraction"
clone_repo \
    https://github.com/reprojection-calibration/${name}.git \
    1.0.0 \
    "${buildroot}/${name}"
cmake_build \
    "${buildroot}/${name}/code"
cmake_install \
    "${buildroot}/${name}/code"
