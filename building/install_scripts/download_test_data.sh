#!/bin/bash

set -eoux pipefail

apt-get update
apt-get install --no-install-recommends --yes \
    ca-certificates \
    wget
rm --force --recursive /var/lib/apt/lists/*

wget --directory-prefix="/data" "https://vision.in.tum.de/tumvi/calibrated/512_16/dataset-calib-imu4_512_16.bag"