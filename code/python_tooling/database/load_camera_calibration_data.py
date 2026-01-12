from database.load_extracted_targets import add_extracted_targets_df_to_camera_calibration_data, \
    load_extracted_targets_df
from database.load_camera_poses import add_camera_poses_df_to_camera_calibration_data, load_camera_poses_df
from database.load_images import image_df_to_camera_calibration_data, load_images_df


def load_camera_calibration_data(db_path):
    df = load_images_df(db_path)
    data = image_df_to_camera_calibration_data(df)

    df = load_extracted_targets_df(db_path)
    add_extracted_targets_df_to_camera_calibration_data(df, data)

    df = load_camera_poses_df(db_path)
    add_camera_poses_df_to_camera_calibration_data(df, data)

    return data


def get_camera_calibration_data_statistics(data):
    statistics = {}
    for sensor, sensor_data in data.items():
        total_frames = 0
        frames_with_image = 0
        frames_with_extracted_target = 0
        frames_with_initial_pose = 0
        frames_with_optimized_pose = 0

        for frame_id, frame_data in sensor_data.get('frames', {}).items():
            total_frames += 1
            if frame_data.get('image') is not None:
                frames_with_image += 1
            if frame_data.get('extracted_target'):
                frames_with_extracted_target += 1
            if frame_data.get('poses'):
                if frame_data['poses'].get('initial'):
                    frames_with_initial_pose += 1
                if frame_data['poses'].get('optimized'):
                    frames_with_optimized_pose += 1

        statistics[sensor] = {
            'total_frames': total_frames,
            'frames_with_image': frames_with_image,
            'frames_with_extracted_target': frames_with_extracted_target,
            'frames_with_initial_pose': frames_with_initial_pose,
            'frames_with_optimized_pose': frames_with_optimized_pose
        }

    return statistics


def get_indexable_timestamp_record(data):
    timestamp_record = {}
    for sensor, sensor_data in data.items():
        timestamps = list(sensor_data['frames'].keys())

        timestamp_record[sensor] = timestamps

    return timestamp_record
