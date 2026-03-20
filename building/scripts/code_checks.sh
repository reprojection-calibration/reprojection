#!/bin/bash

set -eoux pipefail

find /temporary/building -iname '*.sh' -print0 | xargs --null shellcheck

for dir in /temporary/code/library /temporary/code/ros2_ws/src; do
    find "${dir}" \( -iname '*.cpp' -o -iname '*.hpp' -o -iname '*.c' -o -iname '*.h' \) -print0 | xargs --null clang-format --dry-run --Werror
done

# TODO(Jack): Do not manually specify includes!
cppcheck /temporary/code/library --enable=all --error-exitcode=1 --inline-suppr --suppress=missingIncludeSystem \
  -I /temporary/code/library/application/include \
  -I /temporary/code/library/caching/include \
  -I /temporary/code/library/calibration/include \
  -I /temporary/code/library/calibration/src \
  -I /temporary/code/library/calibration_data_views/include \
  -I /temporary/code/library/config/include \
  -I /temporary/code/library/database/include \
  -I /temporary/code/library/database/src \
  -I /temporary/code/library/demos/include \
  -I /temporary/code/library/eigen_utilities/include \
  -I /temporary/code/library/feature_extraction/include \
  -I /temporary/code/library/feature_extraction/src \
  -I /temporary/code/library/geometry/include \
  -I /temporary/code/library/optimization/include \
  -I /temporary/code/library/optimization/src \
  -I /temporary/code/library/pnp/include \
  -I /temporary/code/library/pnp/src \
  -I /temporary/code/library/projection_functions/include \
  -I /temporary/code/library/spline/include \
  -I /temporary/code/library/spline/src \
  -I /temporary/code/library/testing_mocks/include \
  -I /temporary/code/library/testing_utilities/include \
  -I /temporary/code/library/types/include

cppcheck /temporary/code/ros2_ws/src --enable=all --error-exitcode=1 --inline-suppr --suppress=missingIncludeSystem \
  -I /temporary/code/ros2_ws/src/reprojection/include