#!/bin/bash

set -eou pipefail

# NOTE(Jack): THis will install a python virtual environment on your host system! Things you should be aware of:
#   1) Only tested on ubuntu 24.04
#   2) Apt installs system packages (requires sudo)
#   3) Install pyenv and builds and install a python venv
#   4) Creates a directory "wheels/" and copies a python wheel into it

PYENV_ROOT="${PYENV_ROOT:-${HOME}/.pyenv}"
PATH="${PYENV_ROOT}/bin:${PATH}"
project_root="$(dirname "$(realpath -s "$0")")/../.."

# Install apt dependencies and pyenv to host.
sudo "${project_root}/building/install_scripts/apt_install_pyenv_deps.sh"
"${project_root}/building/install_scripts/install_pyenv.sh"

sudo apt-get update
sudo apt-get install --no-install-recommends --yes \
  libgoogle-glog-dev \
  sqlite3

# Build and install the venv.
reprojection_venv="${HOME}/.venv-reprojection"
PYENV_VERSION=3.12 \
  pyenv exec python -m venv "${reprojection_venv}"
"${reprojection_venv}/bin/python" \
  -m pip install -r "${project_root}/building/scripts/requirements-reprojection-env.txt"

# Install our custom python tooling code into the venv.
EDITABLE_INSTALL=1 \
PACKAGE_DIRECTORY="${project_root}/code/python_tooling" \
PROTO_DIRECTORY="${project_root}/code/resources/proto" \
REPROJECTION_SQL_PYTHON_DIR="${project_root}/code/resources/sql" \
RUN_TESTS=0 \
REPROJECTION_VENV="${reprojection_venv}" \
  "${project_root}/building/scripts/build_python_tooling.sh"

# Build the python bindings in a docker image and install them into our venv.
"${project_root}/building/local/build_image.sh" \
  --stage python-binding

mkdir --parents "${project_root}/wheels"
container_id=$(docker create reprojection:python-binding)
docker cp "${container_id}:/buildroot/wheels/." ./wheels/
docker rm "${container_id}"

"${reprojection_venv}/bin/python" -m pip install --force-reinstall ./wheels/*.whl

echo "Run 'source ${reprojection_venv}/bin/activate' to source the environment"