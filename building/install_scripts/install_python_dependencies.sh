#!/bin/bash

set -eoux pipefail

# Required system dependencies for working with python in a venv
apt-get update
apt-get install --no-install-recommends --yes \
    python3 \
    python3-venv

# Create and activate virtual environment
python3 -m venv "${VENV_DIR}"
source "${VENV_DIR}/bin/activate"

python3 -m pip install \
 build \
 numpy \
 setuptools

# Protobuf deps
python3 -m pip install \
  grpcio \
  grpcio-tools \
  protobuf
