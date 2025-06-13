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

FROM base-stage AS build-stage

ARG BUILD_LIBRARY=building/scripts/build_library.sh
RUN --mount=type=bind,source=${BUILD_LIBRARY},target=/temporary/${BUILD_LIBRARY} \
    --mount=type=bind,source=code/,target=/temporary/code \
    /temporary/${BUILD_LIBRARY}


FROM base-stage AS development-stage

RUN apt-get update; apt-get install --no-install-recommends --yes gdb; rm --force --recursive /var/lib/apt/lists/*