#!/bin/bash

set -eoux pipefail

source /temporary/building/install_scripts/source_install_utils.sh

version="1.18.0"
sha256="14fa81bdc34171edb5f1f02b1d60e74802f0439b77fa44e592565d517d72df90"
name="doxygen-${version}.linux.bin.tar.gz"

buildroot="/buildroot"
download_and_extract \
    "https://www.doxygen.nl/files/${name}" \
    "${buildroot}"

install \
  "${buildroot}/doxygen-${version}/bin/doxygen" \
  /usr/local/bin/doxygen

echo "${sha256}  ${buildroot}/${name}" | sha256sum --check --status