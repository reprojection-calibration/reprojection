#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    clang-format \
    cppcheck \
    shellcheck

rm --force --recursive /var/lib/apt/lists/*