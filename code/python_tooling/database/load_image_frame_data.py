from database.load_extracted_targets import load_all_extracted_targets
from database.load_poses import load_poses


def load_image_frame_data(db_path):
    extracted_targets = load_all_extracted_targets(db_path)

    # TODO(Jack): Just put external poses dict in core loading functions
    external_poses = load_poses(db_path, 'external', 'ground_truth')


    # for sensor, frames in extracted_targets.items():
     #   print(frames)
