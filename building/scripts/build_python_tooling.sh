#!/bin/bash

set -eoux pipefail

# ERROR(Jack): Copy pasted from install_python_deps script!!!
VENV_DIR=~/.reprojection-venv
python3 -m venv "$VENV_DIR"
source "$VENV_DIR/bin/activate"

build_directory=/buildroot/build/python

# TODO(Jack): Can we do this all "out of source"? Or do we need to change the directory?
cd /temporary/code/python_tooling

python3 -m grpc_tools.protoc -I ../proto --python_out=generated ../proto/*.proto

# TODO(Jack): Is this the real best way to do this? Do the tests run the tests that we actually want to run in the build
# folder? Or are they run from the soruce code?
pip install -e .
python3 -m unittest discover -s tests -v

