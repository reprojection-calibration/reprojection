#!/bin/bash

set -eoux pipefail

find /temporary/building -iname '*.sh' -print0 | xargs --null shellcheck

find /temporary/code \( -iname '*.cpp' -o -iname '*.hpp' -o -iname '*.c' -o -iname '*.h' \) -print0 | xargs --null clang-format --dry-run --Werror
cppcheck /temporary/code --enable=all --error-exitcode=1 -I /temporary/code/include -I /temporary/code/src --suppress=missingIncludeSystem