#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    python3-build \
    python3-pip \
    python3-protobuf
