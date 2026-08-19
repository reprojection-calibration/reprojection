#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    python3.12-venv \
    python3-build \
    python3-pybind11

# TODO(Jack): This is the single dependency which I think comes as a shared object hanger on the projection function
# library. For now we just install this here and expect that any user needs to have this on their system. That there are
# not more dependencies surprises me! We escaped shared object dependency hell... for now -_-
apt-get install --no-install-recommends --yes \
    libgoogle-glog-dev

rm --force --recursive /var/lib/apt/lists/*