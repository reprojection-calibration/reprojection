#!/bin/bash

set -eoux pipefail

VENV_DIR=${VENV_DIR:-~/.reprojection_venv}

# shellcheck disable=SC1090
source ~/.bash_profile
pyenv install 3.12
pyenv shell 3.12

python3.12 -m venv "${VENV_DIR}"

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python3 -m pip install \
 build \
 dash \
 # TODO(Jack): We use this 2021 version of Kaleido because it does not require a google chrome installation for
 # rendering. Does this introduce any risks or problems for us?
 # shellcheck disable=SC1097
 kaleido==0.2.1 \
 numpy \
 pandas \
 plotly \
 reportlab \
 scipy \
 setuptools

# Protobuf deps
python3 -m pip install \
  grpcio \
  grpcio-tools \
  protobuf



