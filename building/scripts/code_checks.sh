#!/bin/bash

set -eoux pipefail

find /temporary/building -iname '*.sh' -print0 | xargs --null shellcheck

find /temporary/code \( -iname '*.cpp' -o -iname '*.hpp' -o -iname '*.c' -o -iname '*.h' \) -print0 | xargs --null clang-format --dry-run --Werror
cppcheck /temporary/code --enable=all --error-exitcode=1 --suppress=missingIncludeSystem \
  -I /temporary/code/feature-extraction/include \
  -I /temporary/code/feature-extraction/src \
  -I /temporary/code/geometry/include \
  -I /temporary/code/pnp/include \
  -I /temporary/code/pnp/src