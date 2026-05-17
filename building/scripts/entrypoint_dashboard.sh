#!/bin/bash

set -eou pipefail

# shellcheck disable=SC1091
source /buildroot/.reprojection_venv/bin/activate

# TODO(Jack): Do not hardcode the "/workspace" directory!
# TODO(Jack): Allow the user to configure what they are doing here. For example they might not want to generate the
# reports and run the dashboard. This clearly needs to be handled with a little more care and thought.

python3 /temporary/code/python_tooling/report/generate_toml.py
python3 /temporary/code/python_tooling/dashboard/run.py