#!/bin/bash

set -eoux pipefail

BUILD_DIRECTORY=/buildroot/build
code_coverage=OFF

# Adopted from https://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash
for i in "$@"; do
  case $i in
    --code-coverage)
      code_coverage=ON
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
cmake -B "${BUILD_DIRECTORY}" -DCODE_COVERAGE="${code_coverage}" -G Ninja -S /temporary/code
cmake --build "${BUILD_DIRECTORY}"

ctest --output-on-failure --progress --test-dir "${BUILD_DIRECTORY}"