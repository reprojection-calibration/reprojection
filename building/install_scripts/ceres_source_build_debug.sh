#!/bin/bash

set -eoux pipefail

mkdir /buildroot/ceres-debug
cd /buildroot/ceres-debug
cmake /buildroot/ceres-solver-2.2.0 -DBUILD_BENCHMARKS=OFF \
                                    -DBUILD_EXAMPLES=OFF \
                                    -DBUILD_TESTING=OFF \
                                    -DCMAKE_BUILD_TYPE=Debug
make -j4
make install

