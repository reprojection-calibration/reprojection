#!/bin/bash

set -eou pipefail

# shellcheck disable=SC1091
source /buildroot/.reprojection_venv/bin/activate

# TODO(Jack): Do not hardcode the "/workspace" directory! The directory displayed in the dashboard
# should be the one the user passed.
python3 -m dashboard.run