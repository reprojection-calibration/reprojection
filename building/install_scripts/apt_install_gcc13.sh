#!/bin/bash

set -eoux pipefail


apt-get update
apt-get install --no-install-recommends --yes \
    lsb-release

UBUNTU_VERSION=$(lsb_release -rs)
if [[ "$UBUNTU_VERSION" == "20.04" ]] || [[ "$UBUNTU_VERSION" == "22.04" ]]; then
    apt-get install --no-install-recommends --yes \
        gpg-agent \
        software-properties-common
    add-apt-repository ppa:ubuntu-toolchain-r/test
    apt-get update

    apt-get install --no-install-recommends --yes \
        gcc-13 g++-13

    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 \
                            --slave /usr/bin/g++ g++ /usr/bin/g++-13
    update-alternatives --set gcc /usr/bin/gcc-13

    update-alternatives --install /usr/bin/cpp cpp /usr/bin/cpp-13 100
    update-alternatives --set cpp /usr/bin/cpp-13
fi

rm --force --recursive /var/lib/apt/lists/*