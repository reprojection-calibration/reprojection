#!/bin/bash

set -eoux pipefail

tar zxf ceres-solver-2.2.0.tar.gz
mkdir ceres-bin
cd ceres-bin
# WARN(Jack): For building/developing/debugging we need debug symbols, but for a real release make sure to use Release!
cmake ../ceres-solver-2.2.0 -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j3
make test
make install

