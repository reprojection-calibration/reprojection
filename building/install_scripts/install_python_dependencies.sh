#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    python3 \
    python3-venv \
    python3-pip

python3 -m venv "${VENV_DIR}"
source "${VENV_DIR}/bin/activate"


pip install --upgrade \
 build \
 numpy \
 pip \
 setuptools

# Protobuf deps
pip install --upgrade \
  grpcio \
  grpcio-tools \
  protobuf \
  pyasn1 \
  pyasn1-modules
