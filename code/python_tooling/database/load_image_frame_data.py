from database.load_extracted_targets import load_all_extracted_targets
from database.load_poses import load_poses
import pandas as pd
from bisect import bisect_left


# NOTE(Jack): In this function we take advantage of the fact that for image data the foreign key relationships force
# that any camera pose has to match an extracted target - and we timestamp sync our external pose to that time
# TODO(Jack): Only plot external targets if they are available
# TODO(Jack): This function does not have the right name! Align what is actually being loaded here with the name. Just
#  because we are loading data associated with an image frames does not mean it belongs in a data named load_image_frame_data
def load_calibration_poses(db_path):
    # For external poses there is no timestamp matching requirement, so if they are available we just take the closest
    # to the time from our extracted target frames.
    df_external_poses = load_poses(db_path, 'external', 'ground_truth')
    if df_external_poses is None:
        # WARN(Jack): External poses are not technically mandatory!
        return None
    df_external_poses = df_external_poses.sort_values("timestamp_ns")

    return df_external_poses
