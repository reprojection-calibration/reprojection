#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    bc \
    build-essential \
    ca-certificates \
    cmake \
    git \
    lcov \
    ninja-build

# Required for code coverage
apt-get install --no-install-recommends --yes \
    bc \
    lcov

rm --force --recursive /var/lib/apt/lists/*