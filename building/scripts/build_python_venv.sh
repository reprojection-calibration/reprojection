#!/bin/bash

set -eoux pipefail

VENV_DIR=${VENV_DIR:-~/.reprojection_venv}

# TODO(Jack): We need to install a specific version of sqlite manually from source so that we can support modern syntax.
# The current installation script installs it to the /opt/reprojection path because of how we copy source install deps
# into subsequent docker stages. Can we engineer this instead so that here sqlite is just installed into the default
# location so we do not need to specify these things here manually?
export CPPFLAGS="-I/opt/reprojection/include"
export LDFLAGS="-L/opt/reprojection/lib -Wl,-rpath,/opt/reprojection/lib"
export PKG_CONFIG_PATH="/opt/reprojection/lib/pkgconfig"

# shellcheck disable=SC1090
source ~/.bash_profile
pyenv install 3.12
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



