#!/bin/bash

set -eoux pipefail

black --check --exclude '_pb2\.py$' --verbose '/temporary/code/python_tooling'

isort --check --profile black -p dashboard,database,generated '/temporary/code/python_tooling'