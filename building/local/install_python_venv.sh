#!/bin/bash

set -eou pipefail

# NOTE(Jack): THis will install the "reprojection_env" python virtual environment on your host system! Things you should
# be aware of:
#   1) Only tested on ubuntu 24.04
#   2) Creates a .bash_profile file and writes to it (see install_pyenv.sh for details)
#   3) Apt installs system packages (requires sudo)
#   4) Creates a directory "wheels/" and copies a python wheel into it

script_folder="$(dirname "$(realpath -s "$0")")"

"${script_folder}/../install_scripts/install_pyenv.sh"

# NOTE(Jack): Apt installing system packages requires sudo. The reason we do not run the entire local install script as
# sudo is because we don't need too, and then number two it changes the HOME path to /root which the daily user does
# not own.
sudo "${script_folder}/../install_scripts/apt_install_python_venv_deps.sh"

# NOTE(Jack): In the docker workflow we source build sqlite3 because we need to ensure that we have a version that
# supports the "RETURNING" syntax - here we assume that a user is running a modern enough version of ubuntu that the
# apt version is good enough - this will not scale, but I do not really want to source install a different sqlite on the
# users system.
# NOTE(Jack): In the docker workflow libgoogle-glog-dev is installed in the base-stage and is as of now the only dep of
# the bindings which needs to be explicitly installed here (its handled in the base-stage in the dockerized workflow).
sudo apt-get update
sudo apt-get install --no-install-recommends --yes \
  libgoogle-glog-dev \
  sqlite3

"${script_folder}/../scripts/build_python_venv.sh"

venv_dir="${HOME}/.reprojection_venv"

# TODO(Jack): I would like to be able to run the tests here but the bindings are not installed yet! This is the core of
# our python installation problems.
PACKAGE_DIRECTORY="${script_folder}/../../code/python_tooling" \
PROTO_DIRECTORY="${script_folder}/../../code/resources/proto" \
REPROJECTION_SQL_PYTHON_DIR="${script_folder}/../../code/resources/sql" \
RUN_TESTS=0 \
VENV_DIR="${venv_dir}" \
  "${script_folder}/../scripts/build_python_tooling.sh"


# NOTE(Jack): Here is the meat and potatoes. The basic complicating factor of the entire python bindings workflow, and
# why we really need this script in the first place is that we need the python bindings from the library (only the
# projection_functions as of 18.08.2026). As we do not want to even try to compile the library on the user host, we need
# to build them in a docker, then copy them to the host and install them inside the python environment.
# TODO(Jack): Is there some way that we can build the entire environment in docker and then copy that to the host
# system? If we could do that somehow then we could avoid this entire script.
"${script_folder}/../local/build_image.sh" \
  --stage python-binding

mkdir --parents "${script_folder}/../../wheels"
container_id=$(docker create reprojection:python-binding)
docker cp "${container_id}:/buildroot/wheels/." ./wheels/
docker rm "${container_id}"

"${venv_dir}/bin/python3" -m pip install --force-reinstall ./wheels/*.whl

echo "Run 'source ${venv_dir}/bin/activate' to source the environment"