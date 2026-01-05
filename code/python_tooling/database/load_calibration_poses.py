from database.load_extracted_targets import load_all_extracted_targets
from database.load_poses import load_poses


# NOTE(Jack): In this function we take advantage of the fact that for image data the foreign key relationships force
# that any camera pose has to match an extracted target - and we timestamp sync our external pose to that time
# TODO(Jack): Only plot external targets if they are available
# TODO(Jack): This function does not have the right name! Align what is actually being loaded here with the name. Just
#  because we are loading data associated with an image frames does not mean it belongs in a data named load_image_frame_data
def load_calibration_poses(db_path):
    df_initial_camera_poses = load_poses(db_path, 'camera', 'initial')
    if df_initial_camera_poses is not None:
        df_initial_camera_poses = df_initial_camera_poses.sort_values("timestamp_ns")

    df_optimized_camera_poses = load_poses(db_path, 'camera', 'optimized')
    if df_optimized_camera_poses is not None:
        df_optimized_camera_poses = df_optimized_camera_poses.sort_values("timestamp_ns")

    df_external_poses = load_poses(db_path, 'external', 'ground_truth')
    if df_external_poses is not None:
        df_external_poses = df_external_poses.sort_values("timestamp_ns")

    return df_initial_camera_poses, df_optimized_camera_poses, df_external_poses
