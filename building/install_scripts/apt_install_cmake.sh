#!/bin/bash

set -eoux pipefail

# We use c++23 which requires at least CMake 3.20 or higher. Ubuntu 20.04 installs 3.16 by default so we
# need to manually install it here.

apt-get update
apt-get install --no-install-recommends --yes \
    gpg

wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc \
    | gpg --dearmor \
    > /usr/share/keyrings/kitware-archive-keyring.gpg
echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main" \
    > /etc/apt/sources.list.d/kitware.list

apt-get update
apt-get install --no-install-recommends --yes \
    cmake
rm --force --recursive /var/lib/apt/lists/*