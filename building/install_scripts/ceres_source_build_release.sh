#!/bin/bash

set -eoux pipefail

mkdir /buildroot/ceres-release
cd /buildroot/ceres-release
cmake /buildroot/ceres-solver-2.2.0 -DBUILD_BENCHMARKS=OFF \
                                    -DBUILD_EXAMPLES=OFF \
                                    -DBUILD_TESTING=OFF \
                                    -DCMAKE_BUILD_TYPE=Release
make -j4
make install
