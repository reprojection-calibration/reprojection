from database.load_imu_data import imu_data_df_to_imu_calibration_data, load_imu_data_df


def load_imu_calibration_data(db_path):
    df = load_imu_data_df(db_path)
    data = imu_data_df_to_imu_calibration_data(df)

    return data


# TODO(Jack): At the current stage when we are not sure about foreign key relationships, the total_frames will always be
#  the same as frames_with_imu_measurement - but maybe in the future when we add the interpolated poses the concept of
#  total_frames will not even apply here at all?
def calculate_imu_statistics(data):
    statistics = {}
    for sensor, sensor_data in data.items():
        total_frames = 0
        frames_with_imu_measurement = 0

        for frame_id, frame_i in sensor_data.get("frames", {}).items():
            total_frames += 1
            if frame_i.get("imu_measurement") is not None:
                frames_with_imu_measurement += 1

        statistics[sensor] = {
            "total_frames": total_frames,
            "frames_with_imu_measurement": frames_with_imu_measurement,
        }

    return statistics
