import argparse
from pathlib import Path

import cv2
from rosbags.highlevel import AnyReader
from rosbags.image import message_to_cvimage
import numpy as np

# WARN(Jack): There is absolutely no guarantee that this script produces a high quality mp4 video. For the purpose of
# running the application integration tests we just need the mp4 video file and that is it because we cannot even
# extract Kalibr style Aprilgrid features if we wanted to.

parser = argparse.ArgumentParser()
parser.add_argument("image_topic", type=str)
parser.add_argument("input_bag", type=Path)
parser.add_argument("output_mp4", type=str)
args = parser.parse_args()

with AnyReader([args.input_bag]) as reader:
    connections = [c for c in reader.connections if c.topic == args.image_topic]

    writer = None
    frame_count = 0

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)
        frame = message_to_cvimage(msg)

        frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX)
        frame = frame.astype(np.uint8)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        if writer is None:
            height, width = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(args.output_mp4, fourcc, 30.0, (width, height))

        writer.write(frame)
        frame_count += 1

if writer:
    writer.release()

print(f"Wrote {frame_count} frames to {args.output_mp4}")
