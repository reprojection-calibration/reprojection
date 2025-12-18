#!/bin/bash

set -eoux pipefail

# ERROR(Jack): Copy pasted from install_python_deps script!!!
VENV_DIR=~/.reprojection-venv
source "$VENV_DIR/bin/activate"

# TODO(Jack): I would have liked to have the protobuf definition as an integrated part of the package (ex. in setup.py)
#  but I could not get grpc_tools recognized in the setup.py script. Possibly to do with the fact that setup.py does not
# have access to all the packages in the venv. But this is speculation and needs more reasearch.
proto_directory=/temporary/code/proto
package_directory=/temporary/code/python_tooling
python3 -m grpc_tools.protoc -I "${proto_directory}" --python_out="${package_directory}/generated" "${proto_directory}"/*.proto

python3 -m pip install --editable "${package_directory}"
python3 -m unittest discover --start-directory "${package_directory}/tests" --verbose

