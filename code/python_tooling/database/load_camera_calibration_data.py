from database.load_camera_poses import (
    add_camera_poses_df_to_camera_calibration_data,
    load_camera_poses_df,
)
from database.load_extracted_targets import (
    add_extracted_targets_df_to_camera_calibration_data,
    load_extracted_targets_df,
)
from database.load_images import image_df_to_camera_calibration_data, load_images_df
from database.load_reprojection_errors import (
    add_reprojection_errors_df_to_camera_calibration_data,
    load_reprojection_errors_df,
)
from database.types import PoseType


def load_camera_calibration_data(db_path):
    df = load_images_df(db_path)
    data = image_df_to_camera_calibration_data(df)

    df = load_extracted_targets_df(db_path)
    add_extracted_targets_df_to_camera_calibration_data(df, data)

    df = load_camera_poses_df(db_path)
    add_camera_poses_df_to_camera_calibration_data(df, data)

    df = load_reprojection_errors_df(db_path)
    add_reprojection_errors_df_to_camera_calibration_data(df, data)

    return data


def calculate_camera_statistics(data):
    statistics = {}
    for sensor, sensor_data in data.items():
        total_frames = 0
        frames_with_image = 0
        frames_with_extracted_target = 0
        frames_with_initial_pose = 0
        frames_with_initial_reprojection_error = 0
        frames_with_optimized_pose = 0
        frames_with_optimized_reprojection_error = 0

        for frame_id, frame_i in sensor_data.get("frames", {}).items():
            total_frames += 1
            if frame_i.get("image") is not None:
                frames_with_image += 1
            if frame_i.get("extracted_target"):
                frames_with_extracted_target += 1
            if frame_i.get("poses"):
                if frame_i["poses"].get(PoseType.Initial):
                    frames_with_initial_pose += 1
                if frame_i["poses"].get(PoseType.Optimized):
                    frames_with_optimized_pose += 1
            if frame_i.get("reprojection_errors"):
                if frame_i["reprojection_errors"].get(PoseType.Initial):
                    frames_with_initial_reprojection_error += 1
                if frame_i["reprojection_errors"].get(PoseType.Optimized):
                    frames_with_optimized_reprojection_error += 1

        statistics[sensor] = {
            "total_frames": total_frames,
            "frames_with_image": frames_with_image,
            "frames_with_extracted_target": frames_with_extracted_target,
            "frames_with_initial_pose": frames_with_initial_pose,
            "frames_with_initial_reprojection_error": frames_with_initial_reprojection_error,
            "frames_with_optimized_pose": frames_with_optimized_pose,
            "frames_with_optimized_reprojection_error": frames_with_optimized_reprojection_error,
        }

    return statistics


# This has a really complicated name, probably because I have not yet found how to integrate it cleanly into the
# dashboard. But let me quickly explain it. The motivation for this is that dash provides the dcc.Interval which we use
# to get a slider which lets us animate our entire dashboard as a function of time. Our data is however stored and
# mapped by timestamp, and using indexes blindly would not be scalable. Imagine what would happen when a frame has a
# value for some times and not others. If we blindly use an index we might get the wrong value or access out of bounds.
# Therefore, we have this function to simply provide us a sorted list of all the timestamps for each sensor. We can then
# index into this from the dcc.Interval output, and use the retried timestamp to fetch the data we want. If the data
# does not exist then we can handle that easily.
def get_indexable_timestamp_record(data):
    timestamp_record = {}
    for sensor, sensor_data in data.items():
        timestamps = sorted(list(sensor_data["frames"].keys()))

        timestamp_record[sensor] = timestamps

    return timestamp_record
