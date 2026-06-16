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

// TODO(Jack): The CameraState is a type that I regret using. It was designed with the intent that one day in
// the future it would contain the rest of the camera state (ex. extrinsics (?)). But that has not happened yet
// and instead we are left here everytime forced to initialize the struct with the array which seems useless and
// verbose every time we do it. Maybe we start using other fields here soon, or maybe we should eliminate this type all
// together.
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

struct ImuErrorState {
    Vector3d delta_angular_velocity;
    Vector3d delta_linear_acceleration;
};

using ImuError = StampedData<ImuErrorState>;
using ImuErrors = StampedMap<ImuError>;

struct Extrinsic {
    std::string frame_a;
    std::string frame_b;
    Array6d se3_a_b;

    std::string EntityId() const { return EntityId(frame_a, frame_b); }

    static std::string EntityId(std::string_view frame_a, std::string_view frame_b) {
        return "tf_" + std::string(frame_a) + "_xxx_" + std::string(frame_b);
    }
};

// TODO(Jack): Does the existence of this type and its naming make sense at all? Is the gravity term really so closely
// related to the extrinsic calibration that this makes sense?
struct ImuCamExtrinsic {
    Extrinsic tf;
    // TODO(Jack): Does it matter what frame this gravity is in? Or is it implied/true that its always in the world
    // frame? Or does that not matter at all...?
    Array3d gravity;
};

}  // namespace reprojection