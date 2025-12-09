#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    libeigen3-dev \
    libopencv-dev \
    libsqlite3-dev \
    libyaml-cpp-dev

# Required for ceres-solver source build
apt-get update
apt-get install --no-install-recommends --yes \
    libatlas-base-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev

rm --force --recursive /var/lib/apt/lists/*