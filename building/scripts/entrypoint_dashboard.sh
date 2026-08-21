#!/bin/bash

set -eou pipefail

# TODO(Jack): Do not hardcode the "/workspace" directory! The directory displayed in the dashboard
# should be the one the user passed.
"${REPROJECTION_VENV}/bin/python" -m dashboard.run