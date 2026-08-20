#!/bin/bash

set -eoux pipefail

# TODO(Jack): We are shooting ourselves in the foot hard with this python binding environment problem! Ideally we would
# run all the tests every time we build the tooling. The problem we have is that the report generation tooling needs to
# be build for all applications and does not compile the bindings. Therefore we cannot run the binding tests when we are
# only installing the report generation. As I do not know how to selectively run tests simply, we just add a flag to
# turn them off completely. As long as the dashboard is always being built in CI we can sleep ok, but we should still
# probably keep one eye open!
RUN_TESTS=${RUN_TESTS:-1}

# NOTE(Jack): We should set the default values of the environmental variables here and in all places so that a user could
# run all scripts from the project root directory and everything works like it does in the dockerized environment.
PACKAGE_DIRECTORY="${PACKAGE_DIRECTORY:-code/python_tooling}"
PROTO_DIRECTORY="${PROTO_DIRECTORY:-code/resources/proto}"
VENV_DIR="${VENV_DIR:-${HOME}/.reprojection_venv}"

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

# TODO(Jack): I would have liked to have the protobuf definition as an integrated part of the package (ex. in setup.py)
# but I could not get grpc_tools recognized in the setup.py script. Possibly to do with the fact that setup.py does not
# have access to all the packages in the venv. But this is speculation and needs more research.
python3 -m grpc_tools.protoc -I "${PROTO_DIRECTORY}" --python_out="${PACKAGE_DIRECTORY}/generated" "${PROTO_DIRECTORY}"/*.proto

python3 -m pip install "${PACKAGE_DIRECTORY}"

if [[ "${RUN_TESTS}" == "1" ]]; then
    python3 -m unittest discover --start-directory "${PACKAGE_DIRECTORY}" --verbose
fi
