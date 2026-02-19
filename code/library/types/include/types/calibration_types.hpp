#pragma once

#include <map>
#include <string>

#include "types/enums.hpp"
#include "types/sensor_types.hpp"

// TODO(Jack): Make sure the names here to conflict logically with other types.

namespace reprojection {

struct ImageBounds {
    // NOTE(Jack): We made this constexpr so that way it could be used in the testing_utilities
    constexpr ImageBounds(double const _min_width, double const _max_width, double const _min_height,
                          double const _max_height)
        : u_min{_min_width}, u_max{_max_width}, v_min{_min_height}, v_max{_max_height} {}

    ImageBounds() = default;

    double u_min{};
    double u_max{};
    double v_min{};
    double v_max{};
};

struct CameraInfo {
    std::string sensor_name;
    CameraModel camera_model;
    ImageBounds bounds;
};

struct ImuInfo {
    std::string sensor_name;
    // TODO(Jack): Add other non optimized parameterization like known calibration bias or noise properties?
};

struct CalibrationDataset {
    CameraInfo camera;
    std::vector<CameraMeasurement> camera_frames;
    ImuInfo imu;
    std::vector<ImuMeasurement> imu_data;
};

// TODO(Jack): If there is no other foreseeable thing that will be added to the camera state, do we really need a
//  struct? Same idea for FrameState below, but I assume we will have more values coming into FrameState later.
// TODO(Jack): Do maybe the camera-imu extrinsic belong here? I think it makes more sense they would belong to an IMU
//  centric state, but that might never be necessary because we never really optimize the IMU itself in any sense.
struct CameraState {
    ArrayXd intrinsics;
};

struct FrameState {
    Array6d pose;
};

struct OptimizationState {
    CameraState camera_state;
    std::map<uint64_t, FrameState> frames;
};

}  // namespace reprojection