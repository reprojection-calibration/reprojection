#!/bin/bash

set -eoux pipefail

build_directory=/buildroot/build/python

# TODO(Jack): Can we do this all "out of source"? Or do we need to change the directory?
cd /temporary/code/python_tooling

# TODO(Jack): Is this the real best way to do this? Do the tests run the tests that we actually want to run in the build
# folder? Or are they run from the soruce code?
python3 setup.py build --build-base "${build_directory}" --verbose
python3 -m unittest --verbose

