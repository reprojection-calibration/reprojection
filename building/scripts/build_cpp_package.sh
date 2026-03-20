#!/bin/bash

set -eoux pipefail

cmake_build_type=Debug
build_directory=""
src_directory=""
cmake_extra_args=()

# Adopted from https://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
for i in "$@"; do
  case $i in
    --cmake_build_type=*)
      cmake_build_type="${i#*=}"
      shift
      ;;
    --build_directory=*)
      build_directory="${i#*=}"
      shift
      ;;
    --src_directory=*)
      src_directory="${i#*=}"
      shift
      ;;
    --cmake_extra_args=*)
      read -ra cmake_extra_args <<< "${i#*=}"
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

if [[ -z "${build_directory}" ]]; then
  echo "Error: --build_directory is required"
  exit 1
fi

if [[ -z "${src_directory}" ]]; then
  echo "Error: --src_directory is required"
  exit 1
fi

# NOTE(Jack): We append the build type so that we can have Release and Debug builds built in separate folders without
# having to manually specify two different build folders.
build_directory="${build_directory}-${cmake_build_type}"

cmake -B "${build_directory}" \
  -DCMAKE_BUILD_TYPE="${cmake_build_type}" \
  -G Ninja \
  -S "${src_directory}" \
  "${cmake_extra_args[@]}"

cmake --build "${build_directory}"

# NOTE(Jack): The --test-dir option is not supported under cmake version 3.20 - ubuntu 20 defaults to 3.16 - so we just
# do it manually here.
cd "${build_directory}"
ctest --output-on-failure --progress