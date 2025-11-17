#!/bin/bash

set -eoux pipefail

cmake_build_type=Debug

for i in "$@"; do
  case $i in
      --cmake_build_type=*)
      cmake_build_type="${i#*=}"
      shift
      ;;
    -*)
      echo "Unknown option $i"
      exit 1;
      ;;
    *)
      ;;
  esac
done

mkdir /buildroot/ceres-${cmake_build_type}
cd /buildroot/ceres-${cmake_build_type}
cmake /buildroot/ceres-solver-2.2.0 -DBUILD_BENCHMARKS=OFF \
                                    -DBUILD_EXAMPLES=OFF \
                                    -DBUILD_TESTING=OFF \
                                    -DCMAKE_BUILD_TYPE=${cmake_build_type}
make -j4
make install

