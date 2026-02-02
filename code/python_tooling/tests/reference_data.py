from dataclasses import dataclass

from database.types import SensorType


@dataclass
class TestData:
    raw_camera_data: dict
    raw_imu_data: dict
    statistics: dict
    timestamps: dict
    metadata: list


# This is what the callbacks will give us from the dashboard before the data is properly loaded and initialized.
def invalid_data() -> TestData:
    return TestData(
        None,
        None,
        None,
        None,
        None,
    )


# The minimum acceptable versions of datatypes - this should be the minimum state of the data dictionaries at all times
# because the database loading function loaded them properly.
def skeleton_data() -> TestData:
    statistics = {SensorType.Camera: {}, SensorType.Imu: {}}
    timestamps = {SensorType.Camera: {}, SensorType.Imu: {}}

    return TestData(
        raw_camera_data={},
        raw_imu_data={},
        statistics=statistics,
        timestamps=timestamps,
        metadata=[statistics, timestamps],
    )


# This is how the data structure should look assuming we have loaded one sensor - but we do not fill out all subfields
# here!
# NOTE(Jack): A frame can only exist in either raw data frame if at least one valid entry is in the respective base
# data table for that type (ex. the "images" table for the camera calibration). Therefore, we put one key for each
# raw data type, but with no value because it is not mandatory there is actually data to be loaded - at least for the
# camera case this is 100% true, it can be that an image target extraction was attempted but failed, which means that
# the frame exists in the "images" table but nowhere downstream.
def full_data() -> TestData:
    statistics = {
        SensorType.Camera: {"/cam0/image_raw": {"total_frames": 1}},
        SensorType.Imu: {"/imu0": {"total_frames": 1}},
    }
    timestamps = {
        SensorType.Camera: {"/cam0/image_raw": [0]},
        SensorType.Imu: {"/imu0": [0]},
    }

    return TestData(
        raw_camera_data={"/cam0/image_raw": {"frames": {0: {}}}},
        raw_imu_data={"/imu0": {"frames": {0: {}}}},
        statistics=statistics,
        timestamps=timestamps,
        metadata=[statistics, timestamps],
    )
