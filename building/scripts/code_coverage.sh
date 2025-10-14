#!/bin/bash

set -eoux pipefail

# Adopted from https://github.com/svnscha/cpp-coverage-example
lcov --directory /buildroot/build --capture --output-file coverage.info --rc geninfo_auto_base=1 --ignore-errors mismatch,mismatch
lcov --remove coverage.info '/usr/*' '*.test.cpp' '*gtest*' --output-file coverage.filtered.info
genhtml coverage.filtered.info --output-directory coverage-report
