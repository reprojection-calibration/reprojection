from database.load_extracted_targets import load_all_extracted_targets
from database.load_poses import load_poses
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


def load_image_frame_data(db_path):
    # NOTE(Jack): In this function we take advantage of the fact that for image data the foreign key relationships force
    # that any camera pose has to match an extracted target.
    image_frames = load_all_extracted_targets(db_path)

    # For external poses there is no timestamp matching requirement, so if they are available we just take the closest
    # to the time from our extracted target frames.
    external_poses = load_poses(db_path, 'external', 'ground_truth')
    if external_poses is not None:
        # WARN(Jack): Hardcoded '/mocap0'
        external_pose_times = sorted(external_poses['/mocap0'].keys())

        for sensor, frames in image_frames.items():
            for timestamp_i in frames:
                match = closest_timestamp(external_pose_times, timestamp_i)
                image_frames[sensor][timestamp_i]['external_pose'] = external_poses['/mocap0'][match]

    return image_frames
