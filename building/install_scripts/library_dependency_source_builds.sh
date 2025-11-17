#!/bin/bash

set -eoux pipefail

# Install ceres-solver from source, both debug and release targets.
# NOTE(Jack): When we build reprojection with# CMAKE_BUILD_TYPE=Debug it will use the debug ceres build and let us use
# the debugger and when we use CMAKE_BUILD_TYPE=Release it will use the release version of ceres and be fast. It was
# proven that this happens by building ceres as a shared library and using ldd to check what is linked. We cannot check
# directly for the static  (ceres default build type) but it should also be the case.
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