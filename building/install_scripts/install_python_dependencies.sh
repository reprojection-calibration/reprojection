#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    python3 \
    python3-venv \
    python3-pip

VENV_DIR=~/.reprojection-venv
python3 -m venv "$VENV_DIR"
source "$VENV_DIR/bin/activate"

# Build tooling
pip install --upgrade build pip setuptools

# General deps
pip install numpy

# Protobuf deps
pip install grpcio grpcio-tools protobuf pyasn1 pyasn1-modules
