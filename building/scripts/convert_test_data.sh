#!/bin/bash

set -eoux pipefail

# WARN(Jack):

pip install opencv-python-headless rosbags rosbags-image

mkdir /data/ros1 /data/ros2 /data/video_file

rosbags-convert \
    --src /data/dataset-calib-imu4_512_16.bag \
    --dst /data/ros2/dataset-calib-imu4_512_16/

python3 /temporary/building/scripts/ros1_bag_topic_to_mp4.py \
  "/cam0/image_raw" \
  /data/dataset-calib-imu4_512_16.bag \
  /data/video_file/dataset-calib-imu4_512_16.mp4

mv /data/dataset-calib-imu4_512_16.bag /data/ros1/dataset-calib-imu4_512_16.bag