#pragma once

#include <map>
#include <string>

#include "types/enums.hpp"
#include "types/sensor_data_types.hpp"
#include "types/stamped_templates.hpp"

// TODO(Jack): Make sure the names here to conflict logically with other types.

namespace reprojection {

// TODO(Jack): Does this belong in another file named something more camera specific?
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

// TODO(Jack): If there is no other foreseeable thing that will be added to the camera state, do we really need a
//  struct? Same idea for FrameState below, but I assume we will have more values coming into FrameState later.
// TODO(Jack): Do maybe the camera-imu extrinsic belong here? I think it makes more sense they would belong to an IMU
//  centric state, but that might never be necessary because we never really optimize the IMU itself in any sense.
struct CameraState {
    ArrayXd intrinsics;
};

// TODO(Jack): If it turns out we never add anything else to the frame state than we can just remove the struct and use
//  the Array6d directly.
struct FrameState {
    Array6d pose;
};
using Frame = StampedData<FrameState>;
using Frames = StampedMap<Frame>;

struct OptimizationState {
    CameraState camera_state;
    Frames frames;
};

using ReprojectionError = StampedData<ArrayX2d>;
using ReprojectionErrors = StampedMap<ReprojectionError>;

struct ImuInfo {
    std::string sensor_name;
    // TODO(Jack): Add other non optimized parameterization like known calibration bias or noise properties?
};

}  // namespace reprojection