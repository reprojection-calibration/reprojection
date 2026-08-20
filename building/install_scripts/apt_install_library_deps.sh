#!/bin/bash

set -eoux pipefail

apt-get update

apt-get install --no-install-recommends --yes \
    libgtest-dev \
    libopencv-dev \
    libprotobuf-dev \
    libssl-dev \
    protobuf-compiler

# Required for ceres-solver source build
apt-get install --no-install-recommends --yes \
    libatlas-base-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev

# Required to build and test the python bindings
apt-get install --no-install-recommends --yes  \
    gdb  \
    python3-numpy  \
    python3-pybind11

rm --force --recursive /var/lib/apt/lists/*