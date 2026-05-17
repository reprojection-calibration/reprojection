#!/bin/bash

set -eou pipefail

# shellcheck disable=SC1091
source /buildroot/.reprojection_venv/bin/activate

python3 /temporary/code/python_tooling/report/generate_toml.py

python3 /temporary/code/python_tooling/dashboard/run.py