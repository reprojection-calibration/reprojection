#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    libeigen3-dev

rm --force --recursive /var/lib/apt/lists/*