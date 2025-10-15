#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    libceres-dev \
    libeigen3-dev \
    libopencv-dev \
    libyaml-cpp-dev

rm --force --recursive /var/lib/apt/lists/*

# Install april tags from git repository
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
git checkout v3.4.5 # Just the most recent tag - only so that we have a fixed version not because we actually need this one

cmake -B build_april_tag -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build_april_tag --target install