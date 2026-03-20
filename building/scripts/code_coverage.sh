#!/bin/bash

set -eoux pipefail

# Adopted from https://github.com/svnscha/cpp-coverage-example

# NOTE(Jack): We have to manually exclude the /buildroot/build-library-Debug/database here below because protobuf
# causes some problem that makes the coverage tool think that its a source code directory that needs to be checked. This
# only started happening after we refactored the protobuf code to compile directly as source code instead of as a
# separate independent library target. This made the library installation logic more simple and uniform when starting
# to develop the ros1 and ros2 applications, but introduced the requirement for this hack! There must be a better way to
# do this....?
lcov --capture \
     --directory /buildroot/build-library-Debug \
     --exclude /buildroot/build-library-Debug/database \
     --ignore-errors mismatch,mismatch \
     --output-file coverage.info \
     --rc geninfo_auto_base=1

lcov --output-file coverage.filtered.info \
     --remove coverage.info '/usr/*' '*.test.cpp'

genhtml coverage.filtered.info \
        --demangle-cpp \
        --output-directory /buildroot/coverage-report
