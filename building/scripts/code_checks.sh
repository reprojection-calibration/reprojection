#!/bin/bash

set -eoux pipefail

find /temporary/building -iname '*.sh' -print0 | xargs --null shellcheck

find /temporary/code \( -iname '*.cpp' -o -iname '*.hpp' -o -iname '*.c' -o -iname '*.h' \) -print0 | xargs --null clang-format --dry-run --Werror

cppcheck /temporary/code --enable=all --error-exitcode=1 --suppress=missingIncludeSystem \
  -I /temporary/code/calibration/src \
  -I /temporary/code/eigen_utilities/include \
  -I /temporary/code/feature_extraction/include \
  -I /temporary/code/feature_extraction/src \
  -I /temporary/code/geometry/include \
  -I /temporary/code/pnp/include \
  -I /temporary/code/pnp/src \
  -I /temporary/code/projection_functions/include \
  -I /temporary/code/spline/include \
  -I /temporary/code/spline/src \
  -I /temporary/code/testing_mocks/include \
  -I /temporary/code/types/include