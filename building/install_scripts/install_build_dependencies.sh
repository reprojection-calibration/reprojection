#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libgtest-dev \
    ninja-build \
    wget

# Required for code coverage
apt-get install --no-install-recommends --yes \
    bc \
    lcov \
    libjson-xs-perl

rm --force --recursive /var/lib/apt/lists/*