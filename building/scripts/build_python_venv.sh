#!/bin/bash

set -eoux pipefail

VENV_DIR=${VENV_DIR:-${HOME}/.reprojection_venv}

# TODO(Jack): For some reason if we do not do this then sqlite (which we kinda source install) can be found/linked
# against. I thought I had solved this with ldconfig but I guess not - this gets the job done on 20.04 and 24.04
export LDFLAGS="-L/usr/local/lib -Wl,-rpath,/usr/local/lib"

# shellcheck disable=SC1090
source ~/.bash_profile
pyenv install --skip-existing 3.12
pyenv shell 3.12

python3.12 -m venv "${VENV_DIR}"

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

 # TODO(Jack): We use this 2021 version of Kaleido because it does not require a google chrome installation for
 # rendering. Does this introduce any risks or problems for us?
 # shellcheck disable=SC1097
python3 -m pip install \
 build \
 dash \
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



