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

FROM base-stage AS code-coverage-stage

# General code coverage notes:
#
# The solution here was adopted from https://github.com/reprojection-calibration/cpp-coverage-example
#
# The first thing which is a little suprising is that we build the library twice, once in the build-stage and once here
# in the code-coverage-stage. This is because to build for code coverage you have to disable all optimizations,
# therefore in the CMakeLists.txt we added the logic around the CODE_COVERAGE variable. That is used to set the settings
# which are required for code coverage to work. That being said, because we do not have a release stage or any other use
# of the artifacts from the build stage, we could have put this directly into the build stage and just made our regular
# build there be code coverage compliant. However in all the repos so far we have a build-stage that just builds and
# tests the project, therefore I left that as is. This is an arbitrary distinction, and something which we can remove
# as we move forward.
#
# A difference from the tutorial we adopted this from, is that instead of doing it all in the github action, I wanted to
# do it here in docker stages exactly like the build-stage or code-checks-stage. And to be perfectly honest I could not
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
#
# TODO(Jack): One thing that I am still not sure about is if LCOV_EXCL_LINE works for the html generated report

ARG BUILD_LIBRARY=building/scripts/build_library.sh
RUN --mount=type=bind,source=${BUILD_LIBRARY},target=/temporary/${BUILD_LIBRARY} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${BUILD_LIBRARY} --code-coverage

ARG CODE_COVERAGE=building/scripts/code_coverage.sh
RUN --mount=type=bind,source=${CODE_COVERAGE},target=/temporary/${CODE_COVERAGE} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_COVERAGE}

FROM nginx AS code-coverage-report-stage

# Open http://localhost:8080 to view the report
COPY --from=code-coverage-stage  /buildroot/coverage-report  /usr/share/nginx/html

FROM code-coverage-stage AS code-coverage-assertion-stage

ARG CODE_COVERAGE_ASSERTION=building/scripts/code_coverage_assertion.sh
RUN --mount=type=bind,source=${CODE_COVERAGE_ASSERTION},target=/temporary/${CODE_COVERAGE_ASSERTION} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${CODE_COVERAGE_ASSERTION}

FROM base-stage AS build-stage

ARG BUILD_LIBRARY=building/scripts/build_library.sh
RUN --mount=type=bind,source=${BUILD_LIBRARY},target=/temporary/${BUILD_LIBRARY} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${BUILD_LIBRARY}


FROM base-stage AS development-stage

RUN apt-get update; apt-get install --no-install-recommends --yes gdb; rm --force --recursive /var/lib/apt/lists/*