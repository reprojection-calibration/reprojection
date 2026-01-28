FROM ubuntu:24.04 AS configuration-stage

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]
WORKDIR /buildroot


FROM configuration-stage AS base-stage

ARG INSTALL_BUILD_DEPENDENCIES=building/install_scripts/install_build_dependencies.sh
RUN --mount=type=bind,source=${INSTALL_BUILD_DEPENDENCIES},target=/temporary/${INSTALL_BUILD_DEPENDENCIES} \
    /temporary/${INSTALL_BUILD_DEPENDENCIES}

ARG INSTALL_LIBRARY_DEPENDENCIES=building/install_scripts/install_library_dependencies.sh
RUN --mount=type=bind,source=${INSTALL_LIBRARY_DEPENDENCIES},target=/temporary/${INSTALL_LIBRARY_DEPENDENCIES} \
    /temporary/${INSTALL_LIBRARY_DEPENDENCIES}

ARG LIBRARY_DEPENDENCY_SOURCE_BUILDS=building/install_scripts/library_dependency_source_builds.sh
ARG CERES_SOURCE_BUILD=building/install_scripts/ceres_source_build.sh
RUN --mount=type=bind,source=${LIBRARY_DEPENDENCY_SOURCE_BUILDS},target=/temporary/${LIBRARY_DEPENDENCY_SOURCE_BUILDS} \
    --mount=type=bind,source=${CERES_SOURCE_BUILD},target=/temporary/${CERES_SOURCE_BUILD} \
    /temporary/${LIBRARY_DEPENDENCY_SOURCE_BUILDS}


FROM configuration-stage AS code-checks-stage

ARG INSTALL_CODE_CHECK_DEPENDENCIES=building/install_scripts/install_code_check_dependencies.sh
RUN --mount=type=bind,source=${INSTALL_CODE_CHECK_DEPENDENCIES},target=/temporary/${INSTALL_CODE_CHECK_DEPENDENCIES} \
    /temporary/${INSTALL_CODE_CHECK_DEPENDENCIES}

ARG CODE_CHECKS=building/scripts/code_checks.sh
RUN --mount=type=bind,source=${CODE_CHECKS},target=/temporary/${CODE_CHECKS} \
    --mount=type=bind,source=.clang-format,target=/temporary/.clang-format \
    --mount=type=bind,source=building/,target=/temporary/building \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_CHECKS}


# NOTE(Jack): Python venvs can be heavy, confusing, and hard to handle sometimes. As an expedient to get the python
# code checks up and running with the minumum viable dependencies we simply inherit from a default python base image
# and install what we need to get the code checks done. Unintentionally this caused problems with isort not being able
# to disinguish between "first party" and "third party" packages. But we solve it, please see the note in
# code_checks_python.sh for more details.
FROM python:3.12-slim AS code-checks-python-stage

RUN pip install black isort

ARG CODE_CHECKS_PYTHON=building/scripts/code_checks_python.sh
RUN --mount=type=bind,source=${CODE_CHECKS_PYTHON},target=/temporary/${CODE_CHECKS_PYTHON} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_CHECKS_PYTHON}


FROM base-stage AS python-tooling-stage

ENV VENV_DIR="~/.reprojection_venv"

ARG INSTALL_PYENV=building/install_scripts/install_pyenv.sh
RUN --mount=type=bind,source=${INSTALL_PYENV},target=/temporary/${INSTALL_PYENV} \
    /temporary/${INSTALL_PYENV}

# TODO(Jack): Should these go into the base stage? Or is the python workflow unique enough?
ARG INSTALL_PYTHON_DEPENDENCIES=building/install_scripts/install_python_dependencies.sh
RUN --mount=type=bind,source=${INSTALL_PYTHON_DEPENDENCIES},target=/temporary/${INSTALL_PYTHON_DEPENDENCIES} \
    /temporary/${INSTALL_PYTHON_DEPENDENCIES}


COPY code/python_tooling /temporary/code/python_tooling

# TODO(Jack): We want to have a single source of truth for the sql files. Therefore we symlinked the `code/sql` directory
# to the `python_tooling/database/sql` directory. Unfortunately in the dockerized python testing workflow this does not
# work! Locally it works fine, but in the docker build we get the error "ModuleNotFoundError: No module named
# 'database.sql'". To Test that this is the case I copied the sql files directly into the database package and then the
# dockerized workflow worked no problem.
#
# This copy command copies the sql files over the symlinked directory in the python package and then the testing can
# find the files no problem. This is technically acceptable because the "single source of truth" rule is only violated
# here ephermally in the build process. But if there was just a way to make it work without this and simply with the the
# symlinks, like it works for local testing, that would be great.
COPY code/sql /temporary/code/python_tooling/database/sql

ARG BUILD_PYTHON_TOOLING=building/scripts/build_python_tooling.sh
RUN --mount=type=bind,source=${BUILD_PYTHON_TOOLING},target=/temporary/${BUILD_PYTHON_TOOLING} \
    --mount=type=bind,source=code/proto,target=/temporary/code/proto \
    --mount=type=bind,source=code/test_data,target=/temporary/code/test_data \
    /temporary/${BUILD_PYTHON_TOOLING}


FROM base-stage AS debug-build-stage

# General code coverage notes:
#
# The solution here was adopted from https://github.com/reprojection-calibration/cpp-coverage-example
#
# A difference from the tutorial we adopted this from, is that instead of doing it all in the github action, I wanted to
# do it here in docker stages. And to be perfectly honest I could not
# quickly figure out how to do it like he did it in the reference repo, and this should be improved. For example in the
# reference repo the html report is saved as an artefact which can be viewed on github, which is awesome and currently
# not the case here. Instead we have two paths, (1) the code-coverage-report-stage which can be run and used to view the
# report locally and (2) the code-coverage-assertion-stage which is executed in CI (and of course can be locally as
# well) and will fail if the code coverage is not 100%. Somehow it would be nice if we could align this in one docker
# stage, but the need to be able to view the logs in the built image means that we cannot have the stage fail before
# the build is finished.
#
# Another problem that is related to lcov directly, are some false positives on line with curly brackets after return
# statements. I am not the first person to face this problem (https://github.com/linux-test-project/lcov/issues/129) and
# there might already be a solution, but I took the shortcut and just added some LCOV_EXCL_LINE suppression statements
# to get past the problem.

ARG BUILD_LIBRARY=building/scripts/build_library.sh
RUN --mount=type=bind,source=${BUILD_LIBRARY},target=/temporary/${BUILD_LIBRARY} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${BUILD_LIBRARY} --cmake_build_type=Debug

ARG CODE_COVERAGE=building/scripts/code_coverage.sh
RUN --mount=type=bind,source=${CODE_COVERAGE},target=/temporary/${CODE_COVERAGE} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_COVERAGE}


FROM debug-build-stage AS code-coverage-assertion-stage

ARG CODE_COVERAGE_ASSERTION=building/scripts/code_coverage_assertion.sh
RUN --mount=type=bind,source=${CODE_COVERAGE_ASSERTION},target=/temporary/${CODE_COVERAGE_ASSERTION} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_COVERAGE_ASSERTION}


# NOT EXECUTED IN CI!!!
FROM nginx:1.29.2 AS code-coverage-report-stage

# Open http://localhost:8080 to view the report
COPY --from=debug-build-stage  /buildroot/coverage-report  /usr/share/nginx/html


# NOTE(Jack): In the CI pipeline and for local development we often need debugging and code coverage, therefore we do
# Debug builds. However for applications we want them to be as fast as possible so we need a Release type build. We add
# this stage to host that build so that it can be used by all applications.
# NOT EXECUTED IN CI!!!
FROM base-stage AS release-build-stage

ARG BUILD_LIBRARY=building/scripts/build_library.sh
RUN --mount=type=bind,source=${BUILD_LIBRARY},target=/temporary/${BUILD_LIBRARY} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${BUILD_LIBRARY} --cmake_build_type=Release


# NOT EXECUTED IN CI!!!
FROM release-build-stage AS feature-extraction-demo-stage

# TODO(Jack): We should actually use cmakes install capability instead of just copying over the entire build folder!
COPY --from=release-build-stage /buildroot/build /buildroot/build

# NOTE(Jack): This requires that the folder having the config is properly mounted as a volume in the run command
ENTRYPOINT ["/buildroot/build/demos/demos.feature_extraction", "-c", "/temporary/code/library/demos/examples/target_config.toml"]


# NOT EXECUTED IN CI!!!
FROM base-stage AS development-stage

# We only need the ceres debug build if we are developing locally want to step through the code with a debugger.
ARG CERES_SOURCE_BUILD=building/install_scripts/ceres_source_build.sh
RUN --mount=type=bind,source=${CERES_SOURCE_BUILD},target=/temporary/${CERES_SOURCE_BUILD} \
    /temporary/${CERES_SOURCE_BUILD} --cmake_build_type=Debug

# MOVE PROTOBUF INSTALL
RUN set -eoux pipefail; apt-get update; apt-get install --no-install-recommends --yes gdb; rm --force --recursive /var/lib/apt/lists/*