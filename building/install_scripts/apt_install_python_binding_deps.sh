#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    python3.12-venv \
    python3-build \
    python3-pybind11

rm --force --recursive /var/lib/apt/lists/*