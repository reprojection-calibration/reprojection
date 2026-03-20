ARG BASE_IMAGE=ubuntu:24.04
FROM ${BASE_IMAGE} AS configuration-stage

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]
WORKDIR /buildroot


FROM configuration-stage AS apt-base-stage

ARG APT_INSTALL_BASE_DEPS=building/install_scripts/apt_install_base_deps.sh
RUN --mount=type=bind,source=${APT_INSTALL_BASE_DEPS},target=/temporary/${APT_INSTALL_BASE_DEPS} \
    /temporary/${APT_INSTALL_BASE_DEPS}

ARG APT_INSTALL_LIBRARY_DEPS=building/install_scripts/apt_install_library_deps.sh
RUN --mount=type=bind,source=${APT_INSTALL_LIBRARY_DEPS},target=/temporary/${APT_INSTALL_LIBRARY_DEPS} \
    /temporary/${APT_INSTALL_LIBRARY_DEPS}

# NOTE(Jack): We use c++ 20 in the library and we want to support source builds on ubuntu 20 (for ROS Noble) and on
# ubuntu 24 (ROS 2 and all other development). The default gcc on ubuntu 24 is gcc13 (which supports full c++ 20 by
# default) so we also install gcc13 and set it as the primary compiler on ubuntu 20 and 22.
# WARN(Jack): Code coverage does not work ubuntu 20 and 22! There is some problem with gcov which can be fixed easily in
# the apt_install_gcc13.sh script, but there is a more complicated problem that has to do with perl packages and other
# non-trivial version issues. Because we will only run the the full build/debug/test/coverage pipeline on our ubuntu 24
# development we ignore this for now. But if you try to run code coverage on another base image build, it will probably
# not work!
ARG APT_INSTALL_GCC13=building/install_scripts/apt_install_gcc13.sh
RUN --mount=type=bind,source=${APT_INSTALL_GCC13},target=/temporary/${APT_INSTALL_GCC13} \
    /temporary/${APT_INSTALL_GCC13}


# NOTE(Jack): We introduce this source build base stage to guarantee that the artefacts from building are only brought
# into the docker workflow via the /usr/local folder. All other build artefacts will be left behind in this stage.
# WARN(Jack): If in future an apt package is installed here, this will not end up in base-stage because we only copy
# /usr/local here!
FROM apt-base-stage AS source-base-stage

ARG SOURCE_INSTALL_UTILS=building/install_scripts/source_install_utils.sh
ARG SOURCE_INSTALL_LIBRARY_DEPS=building/install_scripts/source_install_library_deps.sh
RUN --mount=type=bind,source=${SOURCE_INSTALL_UTILS},target=/temporary/${SOURCE_INSTALL_UTILS} \
    --mount=type=bind,source=${SOURCE_INSTALL_LIBRARY_DEPS},target=/temporary/${SOURCE_INSTALL_LIBRARY_DEPS} \
    /temporary/${SOURCE_INSTALL_LIBRARY_DEPS}


FROM apt-base-stage AS base-stage

COPY --from=source-base-stage  /usr/local  /usr/local


FROM configuration-stage AS code-checks-stage

ARG APT_INSTALL_CODE_CHECK_DEPS=building/install_scripts/apt_install_code_check_deps.sh
RUN --mount=type=bind,source=${APT_INSTALL_CODE_CHECK_DEPS},target=/temporary/${APT_INSTALL_CODE_CHECK_DEPS} \
    /temporary/${APT_INSTALL_CODE_CHECK_DEPS}

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

ARG INSTALL_PYENV=building/install_scripts/install_pyenv.sh
RUN --mount=type=bind,source=${INSTALL_PYENV},target=/temporary/${INSTALL_PYENV} \
    /temporary/${INSTALL_PYENV}

ARG APT_INSTALL_PYTHON_VENV_DEPS=building/install_scripts/apt_install_python_venv_deps.sh
RUN --mount=type=bind,source=${APT_INSTALL_PYTHON_VENV_DEPS},target=/temporary/${APT_INSTALL_PYTHON_VENV_DEPS} \
    /temporary/${APT_INSTALL_PYTHON_VENV_DEPS}

ARG BUILD_PYTHON_VENV=building/scripts/build_python_venv.sh
RUN --mount=type=bind,source=${BUILD_PYTHON_VENV},target=/temporary/${BUILD_PYTHON_VENV} \
    /temporary/${BUILD_PYTHON_VENV}

# Copy the code directly in because python has a lot of inplace build artefacts that are hard to keep off the local
# system or run without special permissions that docker cannot provide.
COPY code/python_tooling /temporary/code/python_tooling

# TODO(Jack): We want to have a single source of truth for the sql files. Therefore we symlinked the `code/sql` directory
# to the `code/python_tooling/database/sql` directory. Unfortunately in the dockerized python testing workflow this does not
# work! Locally it works fine, but in the docker build we get the error "ModuleNotFoundError: No module named
# 'database.sql'". To Test that this is the case I copied the sql files directly into the database package and then the
# dockerized workflow worked no problem.
#
# This copy command copies the sql files over the symlinked directory in the python package and then the testing can
# find the files no problem. This is technically acceptable because the "single source of truth" rule is only violated
# here ephermally in the build process. But if there was just a way to make it work without this and simply with the the
# symlinks, like it works for local testing, that would be great.
COPY code/sql /temporary/code/python_tooling/database/sql

ARG DB_PATH=/temporary/code/test_data/dataset-calib-imu4_512_16.db3
ARG PACKAGE_DIRECTORY=/temporary/code/python_tooling
ARG PROTO_DIRECTORY=/temporary/code/proto

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

ARG BUILD_CPP_PACKAGE=building/scripts/build_cpp_package.sh
RUN --mount=type=bind,source=${BUILD_CPP_PACKAGE},target=/temporary/${BUILD_CPP_PACKAGE} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/library/,target=/temporary/code/library \
    --mount=type=bind,source=code/proto/,target=/temporary/code/proto \
    --mount=type=bind,source=code/sql/,target=/temporary/code/sql \
    /temporary/${BUILD_CPP_PACKAGE} --cmake_build_type=Debug \
                                    --build_dir=/buildroot/build-library \
                                    --src_dir=/temporary/code/library



# WARN(Jack): Due to problems with installing gcc13 on ubuntu 20/22 the code coverage does not work. This is future work!
FROM debug-build-stage AS code-coverage-stage

ARG CODE_COVERAGE=building/scripts/code_coverage.sh
RUN --mount=type=bind,source=${CODE_COVERAGE},target=/temporary/${CODE_COVERAGE} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_COVERAGE}


FROM code-coverage-stage AS code-coverage-assertion-stage

ARG CODE_COVERAGE_ASSERTION=building/scripts/code_coverage_assertion.sh
RUN --mount=type=bind,source=${CODE_COVERAGE_ASSERTION},target=/temporary/${CODE_COVERAGE_ASSERTION} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_COVERAGE_ASSERTION}


# NOT EXECUTED IN CI!!!
FROM nginx:1.29.2 AS code-coverage-report-stage

# Open http://localhost:8080 to view the report
COPY --from=code-coverage-stage  /buildroot/coverage-report  /usr/share/nginx/html


# NOTE(Jack): In the CI pipeline and for local development we often need debugging and code coverage, therefore we do
# Debug builds. However for applications we want them to be as fast as possible so we need a Release type build. We add
# this stage to host that build so that it can be used by all applications.
# NOT EXECUTED IN CI!!!
FROM base-stage AS release-build-stage

ARG BUILD_CPP_PACKAGE=building/scripts/build_cpp_package.sh
RUN --mount=type=bind,source=${BUILD_CPP_PACKAGE},target=/temporary/${BUILD_CPP_PACKAGE} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/library/,target=/temporary/code/library \
    --mount=type=bind,source=code/proto/,target=/temporary/code/proto \
    --mount=type=bind,source=code/sql/,target=/temporary/code/sql \
    /temporary/${BUILD_CPP_PACKAGE} --cmake_build_type=Release \
                                    --build_dir=/buildroot/build-library \
                                    --src_dir=/temporary/code/library

# NOT EXECUTED IN CI!!!
FROM release-build-stage AS feature-extraction-demo-stage

# TODO(Jack): We should actually use cmakes install capability instead of just copying over the entire build folder!
COPY --from=release-build-stage /buildroot/build-library-Release /buildroot/build-library-Release

# NOTE(Jack): This requires that the folder having the config is properly mounted as a volume in the run command. We
# could also copy it in here into the stage but then we would need to rebuild everytime we want to change the config...
ENTRYPOINT ["/buildroot/build-library-Release/demos/demos.feature_extraction", "-c", "/temporary/code/library/demos/examples/target_config.toml"]


# NOT EXECUTED IN CI!!!
FROM base-stage AS development-stage

RUN set -eoux pipefail; apt-get update; apt-get install --no-install-recommends --yes gdb; rm --force --recursive /var/lib/apt/lists/*


FROM base-stage AS ros1-base-stage

ENV ROS_HOME=/tmp/.ros

# TODO(jack): We put a hard condition in botht he ROS1 and ROS2 install scripts that the ubuntu version is either 20.04
# or 24.04 respectively. This is a kind of hack to prevent someone using a non-intended base image (ex. trying to
# install ROS1 on ubuntu 24). If this is a real long term strategy is at this time not 100% clear!
ARG INSTALL_ROS1=building/install_scripts/install_ros1.sh
RUN --mount=type=bind,source=${INSTALL_ROS1},target=/temporary/${INSTALL_ROS1} \
    /temporary/${INSTALL_ROS1}


FROM ros1-base-stage AS ros1-application-stage

ARG BUILD_CPP_PACKAGE=building/scripts/build_cpp_package.sh
RUN --mount=type=bind,source=${BUILD_CPP_PACKAGE},target=/temporary/${BUILD_CPP_PACKAGE} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/application_ros1/,target=/temporary/code/application_ros1 \
    /temporary/${BUILD_CPP_PACKAGE} --cmake_build_type=Release \
                                    --build_dir=/buildroot/build-ros1-app \
                                    --src_dir=/temporary/code/application_ros1 \
                                    --cmake_extra_args=-DCMAKE_PREFIX_PATH=/opt/ros/noetic


FROM base-stage AS ros2-base-stage

ARG INSTALL_ROS2=building/install_scripts/install_ros2.sh
RUN --mount=type=bind,source=${INSTALL_ROS2},target=/temporary/${INSTALL_ROS2} \
    /temporary/${INSTALL_ROS2}


FROM ros2-base-stage AS ros2-application-stage

ARG BUILD_CPP_PACKAGE=building/scripts/build_cpp_package.sh
RUN --mount=type=bind,source=${BUILD_CPP_PACKAGE},target=/temporary/${BUILD_CPP_PACKAGE} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/application_ros2/,target=/temporary/code/application_ros2 \
    source /opt/ros/jazzy/setup.bash; \
    /temporary/${BUILD_CPP_PACKAGE} --cmake_build_type=Release \
                                    --build_dir=/buildroot/build-ros2-app \
                                    --src_dir=/temporary/code/application_ros2