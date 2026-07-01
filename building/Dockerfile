ARG BASE_IMAGE=ubuntu:24.04@sha256:186072bba1b2f436cbb91ef2567abca677337cfc786c86e107d25b7072feef0c
ARG PYTHON_3_12_SLIM=python:3.12-slim@sha256:3d5ed973e45820f5ba5e46bd065bd88b3a504ff0724d85980dcd05eab361fcf4

FROM ${BASE_IMAGE} AS configuration-stage

ARG DEBIAN_FRONTEND=noninteractive
ARG SOURCE_INSTALL_UTILS=building/install_scripts/source_install_utils.sh

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


FROM apt-base-stage AS build-ceres-stage

ARG SOURCE_INSTALL_CERES=building/install_scripts/source_install_ceres.sh
RUN --mount=type=bind,source=${SOURCE_INSTALL_UTILS},target=/temporary/${SOURCE_INSTALL_UTILS} \
    --mount=type=bind,source=${SOURCE_INSTALL_CERES},target=/temporary/${SOURCE_INSTALL_CERES} \
    /temporary/${SOURCE_INSTALL_CERES}


FROM apt-base-stage AS build-small-libraries-stage

ARG SOURCE_INSTALL_SMALL_LIBRARIES=building/install_scripts/source_install_small_libraries.sh
RUN --mount=type=bind,source=${SOURCE_INSTALL_UTILS},target=/temporary/${SOURCE_INSTALL_UTILS} \
    --mount=type=bind,source=${SOURCE_INSTALL_SMALL_LIBRARIES},target=/temporary/${SOURCE_INSTALL_SMALL_LIBRARIES} \
    /temporary/${SOURCE_INSTALL_SMALL_LIBRARIES}


FROM apt-base-stage AS base-stage

ENV REPROJECTION_SQL_DIR=code/sql
ENV REPROJECTION_SQL_PYTHON_DIR=/temporary/code/python_tooling/database/sql

COPY --from=build-ceres-stage /opt/reprojection /opt/reprojection
COPY --from=build-small-libraries-stage /opt/reprojection /opt/reprojection


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
FROM ${PYTHON_3_12_SLIM} AS code-checks-python-stage

RUN pip install black isort

ARG CODE_CHECKS_PYTHON=building/scripts/code_checks_python.sh
RUN --mount=type=bind,source=${CODE_CHECKS_PYTHON},target=/temporary/${CODE_CHECKS_PYTHON} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_CHECKS_PYTHON}


FROM apt-base-stage AS python-tooling-stage

# NOTE(Jack): We specify the PYENV_ROOT and VENV_DIR to prevent them from being their default values which will see
# them both be put in the $HOME directory. This locatio cause us a lot of permissions issues when trying to create the
# dashboard release stage so we moved them to non permission limited locations like you see here.
ENV PYENV_ROOT="/opt/pyenv"
ENV VENV_DIR=/buildroot/.reprojection_venv

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
COPY ${REPROJECTION_SQL_DIR} ${REPROJECTION_SQL_PYTHON_DIR}

ARG DB_PATH=/temporary/code/test_data/dataset-calib-imu4_512_16.db3
ARG PACKAGE_DIRECTORY=/temporary/code/python_tooling
ARG PROTO_DIRECTORY=/temporary/code/proto

ARG BUILD_PYTHON_TOOLING=building/scripts/build_python_tooling.sh
RUN --mount=type=bind,source=${BUILD_PYTHON_TOOLING},target=/temporary/${BUILD_PYTHON_TOOLING} \
    --mount=type=bind,source=code/proto,target=/temporary/code/proto \
    --mount=type=bind,source=code/test_data,target=/temporary/code/test_data \
    /temporary/${BUILD_PYTHON_TOOLING}


FROM python-tooling-stage AS dashboard-stage

COPY building/scripts/entrypoint_dashboard.sh /buildroot/entrypoint_dashboard.sh

ENTRYPOINT ["/buildroot/entrypoint_dashboard.sh"]

FROM dashboard-stage AS dashboard-smoke-test-stage

HEALTHCHECK --interval=1s --timeout=5s --retries=5 CMD curl --fail http://0.0.0.0:8050 || exit 1


# The code coverage solution here was adopted from https://github.com/reprojection-calibration/cpp-coverage-example
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
FROM base-stage AS debug-build-stage

RUN --mount=type=bind,source=${SOURCE_INSTALL_UTILS},target=/temporary/${SOURCE_INSTALL_UTILS} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/library/,target=/temporary/code/library \
    --mount=type=bind,source=code/proto/,target=/temporary/code/proto \
    --mount=type=bind,source=${REPROJECTION_SQL_DIR},target=/temporary/${REPROJECTION_SQL_DIR} \
    --mount=type=bind,source=code/test_data/,target=/temporary/code/test_data \
    set -eoux pipefail; \
    source /temporary/${SOURCE_INSTALL_UTILS}; \
    export BUILD_TESTING=ON; \
    export CMAKE_BUILD_TYPE=RelWithDebInfo; \
    cmake_build_install \
        "/buildroot/library-${CMAKE_BUILD_TYPE}" \
        "/temporary/code/library"


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


FROM nginx:1.29.2 AS code-coverage-report-stage

# Open http://localhost:8080 to view the report
COPY --from=code-coverage-stage  /buildroot/coverage-report  /usr/share/nginx/html


FROM base-stage AS development-stage

RUN set -eoux pipefail; apt-get update; apt-get install --no-install-recommends --yes gdb; rm --force --recursive /var/lib/apt/lists/*


# NOTE(Jack): In the CI pipeline and for local development we often need debugging and code coverage, therefore we do
# Debug builds. However for applications we want them to be as fast as possible so we need a Release type build. We add
# this stage to host that build so that it can be used by all applications.
# NOT EXECUTED IN CI!!!
FROM base-stage AS library-release-build-stage

RUN --mount=type=bind,source=${SOURCE_INSTALL_UTILS},target=/temporary/${SOURCE_INSTALL_UTILS} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/library/,target=/temporary/code/library \
    --mount=type=bind,source=code/proto/,target=/temporary/code/proto \
    --mount=type=bind,source=${REPROJECTION_SQL_DIR},target=/temporary/${REPROJECTION_SQL_DIR} \
    --mount=type=bind,source=code/test_data/,target=/temporary/code/test_data \
    set -eoux pipefail; \
    source /temporary/${SOURCE_INSTALL_UTILS}; \
    export BUILD_TESTING=OFF; \
    export CMAKE_BUILD_TYPE=Release; \
    cmake_build_install \
        "/buildroot/library-${CMAKE_BUILD_TYPE}" \
        "/temporary/code/library"

# TODO(Jack): This is terrible... What we have here is basically completely copy pasted from library-release-build-stage
# except that here we set INSTALL_EXAMPLES=ON. This all comes from the fact that we have no real packaging solution and
# cannot specify which parts to install where. Instead we have to rebuild the entire thing because in the dependent
# stages we copy the entire /opt/reprojection folder. We need to use something like cpack to solve this and eliminate
# the duplication.
FROM base-stage AS release-build-stage


RUN --mount=type=bind,source=${SOURCE_INSTALL_UTILS},target=/temporary/${SOURCE_INSTALL_UTILS} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/library/,target=/temporary/code/library \
    --mount=type=bind,source=code/proto/,target=/temporary/code/proto \
    --mount=type=bind,source=${REPROJECTION_SQL_DIR},target=/temporary/${REPROJECTION_SQL_DIR} \
    --mount=type=bind,source=code/test_data/,target=/temporary/code/test_data \
    set -eoux pipefail; \
    source /temporary/${SOURCE_INSTALL_UTILS}; \
    export BUILD_TESTING=OFF; \
    export CMAKE_BUILD_TYPE=Release; \
    export INSTALL_EXAMPLES=ON; \
    cmake_build_install \
        "/buildroot/library-${CMAKE_BUILD_TYPE}" \
        "/temporary/code/library"


# NOTE(Jack): We isolate the data download here so that under no circumstance is it possible to bust the cash and force
# a redownload, unless of course you change the download script itself.
FROM ubuntu:24.04@sha256:186072bba1b2f436cbb91ef2567abca677337cfc786c86e107d25b7072feef0c AS test-data-download-stage

ARG DOWNLOAD_TEST_DATA=building/install_scripts/download_test_data.sh
RUN --mount=type=bind,source=${DOWNLOAD_TEST_DATA},target=/temporary/${DOWNLOAD_TEST_DATA} \
    /temporary/${DOWNLOAD_TEST_DATA}


FROM ${PYTHON_3_12_SLIM} AS test-data-stage

COPY --from=test-data-download-stage /data/dataset-calib-imu4_512_16.bag /data/dataset-calib-imu4_512_16.bag

ARG CONVERT_TEST_DATA=building/scripts/convert_test_data.sh
ARG ROS1_BAG_TOPIC_TO_MP4=building/scripts/tum_image_topic_to_mp4.py
RUN --mount=type=bind,source=${CONVERT_TEST_DATA},target=/temporary/${CONVERT_TEST_DATA} \
    --mount=type=bind,source=${ROS1_BAG_TOPIC_TO_MP4},target=/temporary/${ROS1_BAG_TOPIC_TO_MP4} \
    /temporary/${CONVERT_TEST_DATA}

# NOTE(Jack): I normally like to copy or mount files in the repository to `/temporary`. Consistently doing this means
# that you could mount the repository to a docker container at `/temporary` and all scripts/paths etc. would work just
# like they do when run during the build process. However here
COPY code/test_data/dataset-calib-imu4_512_16.db3 /data/ros1/dataset-calib-imu4_512_16.db3
COPY code/test_data/dataset-calib-imu4_512_16.db3 /data/ros2/dataset-calib-imu4_512_16.db3
COPY code/test_data/dataset-calib-imu4_512_16.db3 /data/video_file/dataset-calib-imu4_512_16.db3

ARG UPDATE_FEATURE_EXTRACTION_CACHE=building/scripts/update_feature_extraction_cache_key.py
RUN --mount=type=bind,source=${UPDATE_FEATURE_EXTRACTION_CACHE},target=/temporary/${UPDATE_FEATURE_EXTRACTION_CACHE} \
    python3 /temporary/${UPDATE_FEATURE_EXTRACTION_CACHE}  \
      /data/ros1/dataset-calib-imu4_512_16.db3 ad69955bb5a86463d3cde70f29784adde16ce8f09871d7461592894dbc41a4dc; \
    python3 /temporary/${UPDATE_FEATURE_EXTRACTION_CACHE}  \
      /data/ros2/dataset-calib-imu4_512_16.db3 ad69955bb5a86463d3cde70f29784adde16ce8f09871d7461592894dbc41a4dc; \
    python3 /temporary/${UPDATE_FEATURE_EXTRACTION_CACHE}  \
      /data/video_file/dataset-calib-imu4_512_16.db3 b81580d1ffe01df82b4347f8075a0c20f5f7778e31af901cc2dabd8982493094


FROM apt-base-stage AS ros1-base-stage

ENV ROS_HOME=/tmp/.ros

# TODO(jack): We put a hard condition in both he ROS1 and ROS2 install scripts that the ubuntu version is either 20.04
# or 24.04 respectively. This is a kind of hack to prevent someone using a non-intended base image (ex. trying to
# install ROS1 on ubuntu 24). If this is a real long term strategy is at this time not 100% clear!
ARG INSTALL_ROS1=building/install_scripts/install_ros1.sh
RUN --mount=type=bind,source=${INSTALL_ROS1},target=/temporary/${INSTALL_ROS1} \
    /temporary/${INSTALL_ROS1}

COPY --from=library-release-build-stage /opt/reprojection /opt/reprojection


FROM apt-base-stage AS ros2-base-stage

ARG INSTALL_ROS2=building/install_scripts/install_ros2.sh
RUN --mount=type=bind,source=${INSTALL_ROS2},target=/temporary/${INSTALL_ROS2} \
    /temporary/${INSTALL_ROS2}

COPY --from=library-release-build-stage /opt/reprojection /opt/reprojection


FROM ros1-base-stage AS ros1-release-build-stage

RUN --mount=type=bind,source=${SOURCE_INSTALL_UTILS},target=/temporary/${SOURCE_INSTALL_UTILS} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/application_ros1/,target=/temporary/code/application_ros1 \
    set -eox pipefail; \
    source /opt/ros/noetic/setup.bash; \
    source /temporary/${SOURCE_INSTALL_UTILS}; \
    export BUILD_TESTING=ON; \
    export CMAKE_BUILD_TYPE=Release; \
    cmake_build_install \
        "/buildroot/application_ros1-${CMAKE_BUILD_TYPE}" \
        "/temporary/code/application_ros1"


FROM ros2-base-stage AS ros2-release-build-stage

RUN --mount=type=bind,source=${SOURCE_INSTALL_UTILS},target=/temporary/${SOURCE_INSTALL_UTILS} \
    --mount=type=bind,source=code/cmake_macros/,target=/temporary/code/cmake_macros \
    --mount=type=bind,source=code/application_ros2/,target=/temporary/code/application_ros2 \
    set -eox pipefail; \
    source /opt/ros/jazzy/setup.bash; \
    source /temporary/${SOURCE_INSTALL_UTILS}; \
    export BUILD_TESTING=ON; \
    export CMAKE_BUILD_TYPE=Release; \
    cmake_build_install \
        "/buildroot/application_ros2-${CMAKE_BUILD_TYPE}" \
        "/temporary/code/application_ros2"

FROM ros1-base-stage AS ros1-integration-test-stage

COPY code/test_data/calibration_config.toml /data/calibration_config.toml
COPY --from=test-data-stage /data/ros1 /data
COPY --from=ros1-release-build-stage /opt/reprojection /opt/reprojection

ARG INTEGRATION_TEST=building/scripts/integration_test.sh
RUN --mount=type=bind,source=${INTEGRATION_TEST},target=/temporary/${INTEGRATION_TEST} \
    set -eox pipefail; \
    source /opt/ros/noetic/setup.bash; \
    export APP="/opt/reprojection/bin/application_ros1.reprojection_app"; \
    export DATA="/data/dataset-calib-imu4_512_16.bag"; \
    /temporary/${INTEGRATION_TEST}


FROM ros2-base-stage AS ros2-integration-test-stage

COPY code/test_data/calibration_config.toml /data/calibration_config.toml
COPY --from=test-data-stage /data/ros2 /data
COPY --from=ros2-release-build-stage /opt/reprojection /opt/reprojection

ARG INTEGRATION_TEST=building/scripts/integration_test.sh
RUN --mount=type=bind,source=${INTEGRATION_TEST},target=/temporary/${INTEGRATION_TEST} \
    set -eox pipefail; \
    source /opt/ros/jazzy/setup.bash; \
    export APP="/opt/reprojection/bin/application_ros2.reprojection_app"; \
    export DATA="/data/dataset-calib-imu4_512_16/"; \
    /temporary/${INTEGRATION_TEST}

FROM base-stage AS video-file-integration-test-stage

COPY code/test_data/calibration_config.toml /data/calibration_config.toml
COPY --from=test-data-stage /data/video_file /data
COPY --from=release-build-stage /opt/reprojection /opt/reprojection

ARG INTEGRATION_TEST=building/scripts/integration_test.sh
RUN --mount=type=bind,source=${INTEGRATION_TEST},target=/temporary/${INTEGRATION_TEST} \
    export APP="/opt/reprojection/bin/application.calibration"; \
    export DATA="/data/dataset-calib-imu4_512_16.mp4"; \
    /temporary/${INTEGRATION_TEST}


FROM ros1-base-stage AS ros1-app-stage

# TODO(Jack): These four lines are copy and pasted now in all three release stages, is there anyway to simplify that?
COPY --from=python-tooling-stage /opt/pyenv /opt/pyenv
COPY --from=python-tooling-stage /buildroot/.reprojection_venv /buildroot/.reprojection_venv
COPY --from=python-tooling-stage /temporary/code/python_tooling /temporary/code/python_tooling
COPY ${REPROJECTION_SQL_DIR} ${REPROJECTION_SQL_PYTHON_DIR}

ENV APP_FLAVOR=ros1

COPY --from=ros1-release-build-stage /opt/reprojection /opt/reprojection
COPY building/scripts/entrypoint.sh /buildroot/entrypoint.sh

RUN set -eox pipefail; \
    ln /opt/reprojection/bin/application_ros1.reprojection_app /buildroot/reprojection-calibration-application

ENTRYPOINT ["/buildroot/entrypoint.sh"]


FROM ros2-base-stage AS ros2-app-stage

# TODO(Jack): Copy paste!
COPY --from=python-tooling-stage /opt/pyenv /opt/pyenv
COPY --from=python-tooling-stage /buildroot/.reprojection_venv /buildroot/.reprojection_venv
COPY --from=python-tooling-stage /temporary/code/python_tooling /temporary/code/python_tooling
COPY ${REPROJECTION_SQL_DIR} ${REPROJECTION_SQL_PYTHON_DIR}

# TODO(Jack): Needed? Can we put this in the entrypoint script instead?
ENV PATH="/buildroot/.reprojection_venv/bin:${PATH}"

ENV APP_FLAVOR=ros2

COPY --from=ros2-release-build-stage /opt/reprojection /opt/reprojection
COPY building/scripts/entrypoint.sh /buildroot/entrypoint.sh

RUN set -eox pipefail; \
    ln /opt/reprojection/bin/application_ros2.reprojection_app /buildroot/reprojection-calibration-application

ENTRYPOINT ["/buildroot/entrypoint.sh"]


FROM base-stage AS video-file-app-stage

# TODO(Jack): Copy paste!
COPY --from=python-tooling-stage /opt/pyenv /opt/pyenv
COPY --from=python-tooling-stage /buildroot/.reprojection_venv /buildroot/.reprojection_venv
COPY --from=python-tooling-stage /temporary/code/python_tooling /temporary/code/python_tooling
COPY ${REPROJECTION_SQL_DIR} ${REPROJECTION_SQL_PYTHON_DIR}

ENV APP_FLAVOR=video-file

COPY --from=release-build-stage /opt/reprojection /opt/reprojection
COPY building/scripts/entrypoint.sh /buildroot/entrypoint.sh

RUN set -eox pipefail; \
    ln /opt/reprojection/bin/application.calibration /buildroot/reprojection-calibration-application

ENTRYPOINT ["/buildroot/entrypoint.sh"]


FROM ros1-app-stage AS ros1-app-smoke-test-stage

COPY code/test_data/calibration_config.toml /data/calibration_config.toml
COPY --from=test-data-stage /data/ros1 /data


FROM ros2-app-stage AS ros2-app-smoke-test-stage

COPY code/test_data/calibration_config.toml /data/calibration_config.toml
COPY --from=test-data-stage /data/ros2 /data


FROM video-file-app-stage AS video-file-app-smoke-test-stage

COPY code/test_data/calibration_config.toml /data/calibration_config.toml
COPY --from=test-data-stage /data/video_file /data
