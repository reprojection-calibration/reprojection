#!/bin/bash

set -eoux pipefail

# NOTE(Jack): We should set the default values of the environmental variables here and in all places so that a user could
# run all scripts from the project root directory and everything works like it does in the dockerized environment.
PACKAGE_DIRECTORY=${PACKAGE_DIRECTORY:-code/python_tooling}
PROTO_DIRECTORY=${PROTO_DIRECTORY:-code/proto}
VENV_DIR=${VENV_DIR:-~/.reprojection_venv}

# We need to export DB_PATH as an environmental because it is actually used internally by the python testing to get the
# correct database path.
export DB_PATH=${DB_PATH:-code/test_data/dataset-calib-imu4_512_16.db3}

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

# TODO(Jack): I would have liked to have the protobuf definition as an integrated part of the package (ex. in setup.py)
#  but I could not get grpc_tools recognized in the setup.py script. Possibly to do with the fact that setup.py does not
# have access to all the packages in the venv. But this is speculation and needs more research.
python3 -m grpc_tools.protoc -I "${PROTO_DIRECTORY}" --python_out="${PACKAGE_DIRECTORY}/generated" "${PROTO_DIRECTORY}"/*.proto

python3 -m pip install --editable "${PACKAGE_DIRECTORY}"
python3 -m unittest discover --start-directory "${PACKAGE_DIRECTORY}" --verbose
