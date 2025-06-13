#!/bin/bash

set -eoux pipefail

BUILD_DIRECTORY=/buildroot/build

cmake -B "${BUILD_DIRECTORY}" -G Ninja -S /temporary/code
cmake --build "${BUILD_DIRECTORY}"

ctest --output-on-failure --progress --test-dir "${BUILD_DIRECTORY}"