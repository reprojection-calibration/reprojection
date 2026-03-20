#!/bin/bash

set -eox pipefail

# TODO(Jack): This script is almost exactly the same as the build_library.sh one!

build_directory=/buildroot/build-ros1
cmake_build_type=Debug

# Adopted from https://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
for i in "$@"; do
  case $i in
      --cmake_build_type=*)
      cmake_build_type="${i#*=}"
      shift
      ;;
    -*)
      echo "Unknown option $i"
      exit 1
      ;;
    *)
      ;;
  esac
done

# TODO(Jack): Make release build by default! Note that release builds cannot be built with code coverage enabled!
cmake -B "${build_directory}" -DCMAKE_BUILD_TYPE="${cmake_build_type}"  -G Ninja -S /temporary/code/ros1_ws/src/reprojection -DCMAKE_PREFIX_PATH=/opt/ros/noetic
cmake --build "${build_directory}"


cd "${build_directory}"
ctest --output-on-failure --progress