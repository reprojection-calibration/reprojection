#!/bin/bash

set -eoux pipefail

pip install opencv-python-headless rosbags rosbags-image

# TODO(Jack): Something that we need to be careful about is that we now have these test data paths and names copied all
#  over the place! It would be nice if we designed the system such that they are only specified in one place.
rosbags-convert \
    --src /data/dataset-calib-imu4_512_16.bag \
    --dst /data/dataset-calib-imu4_512_16/

python3 /temporary/building/scripts/ros1_bag_topic_to_mp4.py \
  "/cam0/image_raw" \
  /data/dataset-calib-imu4_512_16.bag \
  /data/dataset-calib-imu4_512_16.mp4