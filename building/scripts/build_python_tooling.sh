#!/bin/bash

set -eoux pipefail

build_directory=/buildroot/build/python

cd /temporary/code/python_tooling

python3 setup.py build --build-base "${build_directory}"

python3 -m unittest --verbose

