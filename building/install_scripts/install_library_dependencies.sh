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


# WARN(Jack): My intention here by installing ceres from source is that we get much better source code navigation with
# clion. The apt package only includes four headers (those required for autodiff) and no debug symbols. Therefore with
# the source install we get much much better source code navigation, even if not 100%.
#
# We also want to be able to use the interactive debugger to step through the ceres source code, which requires that we
# build ceres with "-DCMAKE_BUILD_TYPE=Debug". Ceres explicitly prints out a message that says roughly "Do not build in
# debug, it is SUPER slow". Therefore I decide to also build and install the Release build version which means that we
# end up with two installed configuration and two installed static libraries, the debug one
# "/usr/local/lib/libceres-debug.a" and the release one "/usr/local/lib/libceres.a".
#
# It was my expectation/hope that when we do a release build of the reprojection library that it would automatically go
# select the debug ceres lib, and when we did a release build it would select the release ceres lib. However I cannot
# prove that this is the case, and I have exhausted all avenues. I leave this as a note here for a future person with
# more energy who can investigate this issue.

# Install ceres-solver from source, both debug and release targets.
wget http://ceres-solver.org/ceres-solver-2.2.0.tar.gz

tar zxf ceres-solver-2.2.0.tar.gz
mkdir ceres-debug
cd ceres-debug
cmake ../ceres-solver-2.2.0 -DBUILD_BENCHMARKS=OFF \
                            -DBUILD_EXAMPLES=OFF \
                            -DBUILD_TESTING=OFF \
                            -DCMAKE_BUILD_TYPE=Debug
make -j4
make install

cd ..
mkdir ceres-release
cd ceres-release
cmake ../ceres-solver-2.2.0 -DBUILD_BENCHMARKS=OFF \
                            -DBUILD_EXAMPLES=OFF \
                            -DBUILD_TESTING=OFF \
                            -DCMAKE_BUILD_TYPE=Release
make -j4
make install

# Install april tags from git repository
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
git checkout v3.4.5 # Just the most recent tag - only so that we have a fixed version not because we actually need this one

cmake -B build_april_tag -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build_april_tag --target install