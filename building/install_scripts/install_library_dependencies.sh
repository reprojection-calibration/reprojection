#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev

# Required for ceres-solver source build
apt-get update
apt-get install --no-install-recommends --yes \
    libatlas-base-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev

rm --force --recursive /var/lib/apt/lists/*

# Install ceres-solver from source
wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz

tar zxf ceres-solver-2.2.0.tar.gz
mkdir ceres-bin
cd ceres-bin
# WARN(Jack): For building/developing/debugging we need debug symbols, but for a real release make sure to use Release!
cmake ../ceres-solver-2.2.0 -DCMAKE_BUILD_TYPE=Debug
make -j4
make test
make install

# Install april tags from git repository
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
git checkout v3.4.5 # Just the most recent tag - only so that we have a fixed version not because we actually need this one

cmake -B build_april_tag -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build_april_tag --target install