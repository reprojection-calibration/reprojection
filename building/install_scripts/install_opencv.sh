#!/bin/bash

set -eoux pipefail

cmake_build_type=Debug

git clone --branch "4.13.0" --depth 1  "https://github.com/opencv/opencv.git" "/buildroot/opencv"

mkdir /buildroot/opencv-"${cmake_build_type}"
cd /buildroot/opencv-"${cmake_build_type}"
cmake /buildroot/opencv -DBUILD_SHARED_LIBS=OFF \
                        -DCMAKE_BUILD_TYPE="${cmake_build_type}"
make -j$(nproc)
make install