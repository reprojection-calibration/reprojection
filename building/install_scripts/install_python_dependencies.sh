#!/bin/bash

set -eoux pipefail

source ~/.bash_profile
pyenv shell 3.12.12

export VENV_DIR="~/.reprojection_venv"
# Create and activate virtual environment
python3.12 -m venv "${VENV_DIR}"
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
