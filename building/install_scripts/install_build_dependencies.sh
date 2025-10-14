#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    build-essential \
    ca-certificates \
    cmake \
    git \
    lcov \
    ninja-build

rm --force --recursive /var/lib/apt/lists/*