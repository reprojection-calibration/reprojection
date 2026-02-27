from database.load_extracted_targets import (
    add_extracted_targets_df_to_camera_calibration_data,
    load_extracted_targets_df,
)
from database.load_images import image_df_to_camera_calibration_data, load_images_df
from database.load_poses import (
    add_camera_poses_df_to_camera_calibration_data,
    load_poses_df,
)
from database.load_reprojection_errors import (
    add_reprojection_errors_df_to_camera_calibration_data,
    load_reprojection_errors_df,
)
from database.types import PoseType


def load_camera_calibration_data(db_path):
    df = load_images_df(db_path)
    if df is None:
        return None

    data = image_df_to_camera_calibration_data(df)

    df = load_extracted_targets_df(db_path)
    add_extracted_targets_df_to_camera_calibration_data(df, data)

    df = load_poses_df(db_path)
    add_camera_poses_df_to_camera_calibration_data(df, data)

    df = load_reprojection_errors_df(db_path)
    add_reprojection_errors_df_to_camera_calibration_data(df, data)

    return data


def calculate_camera_statistics(data):
    # TODO(Jack): Should this function be "protected" from the outside instead?
    if data is None:
        return None

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


# Our databases have foreign key constraints which we use to enforce that every single piece of data is associated with
# a frame. For example for the camera data, the "images" table is defined as:
#
#   CREATE TABLE IF NOT EXISTS images (
#       timestamp_ns INTEGER NOT NULL,
#       sensor_name  TEXT    NOT NULL,
#       data         BLOB    NULL,
#   PRIMARY KEY (timestamp_ns, sensor_name));
#
# All subsequent tables in the camera calibration process that calculate data for a frame, here defined by the tuple
# (timestamp_ns, sensor_name), have a foreign key relationship either directly or indirectly to this timestamp and
# sensor name. The timestamps in the "images" table therefore define the entire set of possible timestamps which can
# ever be found in the camera calibration database tables. In this sense this set of timestamps is the "reference."
# Maybe there is a better name but for now this fits!
#
# So in this method here we are iterating over all the possible timestamps and putting them into a sorted list. This
# is then what we use to drive things like the index slider bar, and set axis values for timeseries plots. Having this
# consistency across an entire sensors data display is very nice because it makes it clear what the common timeframe is
# and also when data is missing. Wonderful :)
def get_reference_timestamps(data):
    if data is None:
        return None

    timestamp_record = {}
    for sensor, sensor_data in data.items():
        timestamps_int = sorted(list(sensor_data["frames"].keys()))

        # NOTE(Jack): This is another place where json serialization and types comes back and bites us! Here we are
        # storing timestamps in a python list. When these get handled by dash and put into the metadata store, they are
        # converted to string, because it does not have an int type. If this is not already an int type, we loose
        # precision from the automatic dash casting, and therefore when we decode it for plotting, the timestamps no
        # longer match. Therefore we store cast it here directly to a string, which is then handled easily by dash with
        # no conversion or anything, and then on the consumption side if needed we convert the string back to an int in
        # python which can handle long int with no problem.
        timestamps_str = [str(t) for t in timestamps_int]

        timestamp_record[sensor] = timestamps_str

    return timestamp_record
