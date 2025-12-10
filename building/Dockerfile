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
ENTRYPOINT ["/buildroot/build/demos/demos.feature_extraction", "-c", "/temporary/code/demos/examples/target_config.yaml"]


# NOT EXECUTED IN CI!!!
FROM base-stage AS development-stage

# We only need the ceres debug build if we are developing locally want to step through the code with a debugger.
ARG CERES_SOURCE_BUILD=building/install_scripts/ceres_source_build.sh
RUN --mount=type=bind,source=${CERES_SOURCE_BUILD},target=/temporary/${CERES_SOURCE_BUILD} \
    /temporary/${CERES_SOURCE_BUILD} --cmake_build_type=Debug

# MOVE PROTOBUF INSTALL
RUN set -eoux pipefail; apt-get update; apt-get install --no-install-recommends --yes gdb; rm --force --recursive /var/lib/apt/lists/*