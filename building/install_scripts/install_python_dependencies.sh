#!/bin/bash

set -eoux pipefail

# shellcheck disable=SC1090
source ~/.bash_profile
pyenv shell 3.12.12

python3.12 -m venv "${VENV_DIR}"

# shellcheck disable=SC1091
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
