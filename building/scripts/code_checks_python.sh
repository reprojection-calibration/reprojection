#!/bin/bash

set -eoux pipefail

black --check --exclude '_pb2\.py$' --verbose '/temporary/code/python_tooling'

isort --check '/temporary/code/python_tooling'