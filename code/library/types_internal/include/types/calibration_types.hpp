#pragma once

#include <map>

#include "types/enums.hpp"
#include "types/sensor_data_types.hpp"
#include "types/stamped_templates.hpp"

#include "database_types.hpp"

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
    CameraModel camera_model;
    ImageBounds bounds;
};

// TODO(Jack): One day if we add multi-target calibration we will likely have to add a target ID here. But for now
// (15.5.26) we only support one target at a time therefore we do not require an identifier.
struct TargetInfo {
    TargetType target_type;
    int height;
    int width;
    double unit_dimension;
    // TODO(Jack): This annoys me that this is here because it only actually really applies to the circle grid target.
    // You can have a symmetric or asymmetric circle grid but that is not possible for the other targets. Therefore is
    // an extra piece of information that has no use for most targets. We are missing some abstraction here and if
    // possible we should fix this.
    bool asymmetric;
};

struct Intrinsic {
    ArrayXd value;
};

// TODO(Jack): If it turns out we never add anything else to the frame state than we can just remove the struct and use
//  the Array6d directly.
struct FrameState {
    Array6d pose;
};
using Frame = StampedData<FrameState>;
using Frames = StampedMap<Frame>;

struct OptimizationState {
    Intrinsic camera_state;
    Frames frames;
};

using ReprojectionError = StampedData<ArrayX2d>;
using ReprojectionErrors = StampedMap<ReprojectionError>;

struct ImuErrorState {
    Vector3d delta_angular_velocity;
    Vector3d delta_linear_acceleration;
};

using ImuError = StampedData<ImuErrorState>;
using ImuErrors = StampedMap<ImuError>;

struct Extrinsic {
    AssetId frame_a;
    AssetId frame_b;
    Array6d se3_a_b;
};

}  // namespace reprojection