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
# that any camera pose has to match an extracted target - and we timestamp sync our external pose to that time
# TODO(Jack): Only plot external targets if they are available
# TODO(Jack): This function does not have the right name! Align what is actually being loaded here with the name. Just
#  because we are loading data associated with an image frames does not mean it belongs in a data named load_image_frame_data
def load_image_frame_data(db_path):
    # TODO(Jack): Only load the frame ids instead of the extracted targets here because in this function we do this just
    #  to get the frame_id
    df = load_all_extracted_targets(db_path)
    if df is None:
        return None
    df = df.sort_values("timestamp_ns")

    # TODO(Jack): Formalize the multindex logic here and below. One key point is that plotly cannot work with
    #  multiindexed tables (at least not directly) therefore we need to flatten the multindex into a "precatenated"
    #  single index which is why we get column names like external_pose_rx in app.py.
    # TODO(Jack): Prove that we need multindex at all
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
        # WARN(Jack): External poses are not technically mandatory!
        return None
    df_external_poses = df_external_poses.sort_values("timestamp_ns")

    # NOTE(Jack): Here we only take the closest external pose to the frame_id timestamps. It might be at some future
    # time that we want to plot all external poses
    # TODO(Jack): Maximum allowable tolerance for syncing
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
