#!/bin/bash

set -eoux pipefail

build_directory=/buildroot/build
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
cmake -B "${build_directory}" -DCMAKE_BUILD_TYPE="${cmake_build_type}"  -G Ninja -S /temporary/code
cmake --build "${build_directory}"

ctest --output-on-failure --progress --test-dir "${build_directory}"