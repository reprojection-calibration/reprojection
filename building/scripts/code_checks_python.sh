#!/bin/bash

set -eoux pipefail

black --check --exclude '_pb2\.py$' --verbose '/temporary/code/python_tooling'

# WARN(Jack): This turned into a giant boondoggle. In the isolated docker environment that we use for the python code checks the
# python_tooling packages are not installed (in contrast to probably how you have them locally). This means that getting
# the docker python code checks stage isort execution to match the local development execution was really annoying! The core
# problem in the docker environment was getting isort to recognize our packages (ex. dashboard, database, etc.) as
# "first party" packages. I only ended up getting this to work by adding the .isort.cfg to the python_tooling package
# root. All attempts at configuring the "known first party" packages through the command line were unsuccessful. As more
# packages are added overtime these should be added to the configuration file.
isort --check '/temporary/code/python_tooling'