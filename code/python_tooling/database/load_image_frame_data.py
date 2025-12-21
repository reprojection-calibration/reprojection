from database.load_extracted_targets import load_all_extracted_targets
from database.load_poses import load_poses
import pandas as pd
from bisect import bisect_left


def closest_timestamp(timestamps, timestamp_x):
    if not timestamps:
        return None

    # Find the insertion point for timestamp_x
    pos = bisect_left(timestamps, timestamp_x)

    if pos == 0:
        return timestamps[0]
    if pos == len(timestamps):
        return timestamps[-1]

    before = timestamps[pos - 1]
    after = timestamps[pos]

    # Return whichever is closer
    if abs(after - timestamp_x) < abs(timestamp_x - before):
        return after
    else:
        return before


# NOTE(Jack): In this function we take advantage of the fact that for image data the foreign key relationships force
# that any camera pose has to match an extracted target.
def load_image_frame_data(db_path):
    df = load_all_extracted_targets(db_path)
    if df is None:
        return None
    df = df.sort_values("timestamp_ns")

    tuples = [
        ("frame_id", "timestamp_ns"),
        ("frame_id", "sensor_name"),
        ("extracted_target", "data"),
    ]
    multi_index = pd.MultiIndex.from_tuples(tuples)
    df.columns = multi_index

    # For external poses there is no timestamp matching requirement, so if they are available we just take the closest
    # to the time from our extracted target frames.
    df_external_poses = load_poses(db_path, 'external', 'ground_truth')
    if df_external_poses is None:
        return None
    df_external_poses = df_external_poses.sort_values("timestamp_ns")

    # TODO(Jack): Maximum allowable tolerancce
    df_matched = pd.merge_asof(
        df['frame_id'],
        df_external_poses,
        on="timestamp_ns",
        direction="nearest"
    )

    df_matched = df_matched[['rx', 'ry', 'rz', 'x', 'y', 'z']]
    tuples = [
        ("external_pose", "rx"),
        ("external_pose", "ry"),
        ("external_pose", "rz"),
        ("external_pose", "x"),
        ("external_pose", "y"),
        ("external_pose", "z"),
    ]
    multi_index = pd.MultiIndex.from_tuples(tuples)
    df_matched.columns = multi_index

    df = pd.concat([df, df_matched], axis=1)

    return df
