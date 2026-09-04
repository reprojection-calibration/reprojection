#pragma once

#include "types/enums.hpp"
#include "types/sensor_data_types.hpp"
#include "types/stamped_templates.hpp"

#include "database_types.hpp"

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
    CameraModel camera_model;
    ImageBounds bounds;
};

struct TargetInfo {
    TargetType target_type;
    int height;
    int width;
    double unit_dimension;
    // TODO(Jack): This annoys me that this is here because it only actually really applies to the circle grid target.
    // You can have a symmetric or asymmetric circle grid but that is not possible for the other targets.
    bool asymmetric;
};

struct Intrinsic {
    ArrayXd value;
};

struct Pose {
    Array6d value;
};

using Frame = StampedData<Pose>;
using Frames = StampedMap<Frame>;

// TODO(Jack): Remove once we complete the ba problem refactor!
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