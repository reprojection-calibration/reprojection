#pragma once

#include <map>
#include <optional>
#include <string>

#include "types/algorithm_types.hpp"
#include "types/camera_types.hpp"
#include "types/eigen_types.hpp"

// TODO(Jack): Make sure the names here to conflict logically with other types.

namespace reprojection {

struct ImageBounds {
    ImageBounds(double const _min_width, double const _max_width, double const _min_height, double const _max_height)
        : u_min{_min_width}, u_max{_max_width}, v_min{_min_height}, v_max{_max_height} {}

    double u_min;
    double u_max;
    double v_min;
    double v_max;
};

struct CameraSensorInfo {
    std::string sensor_name;
    CameraModel camera_model;
    ImageBounds bounds;
};

struct CalibrationDataFrame {
    CalibrationDataFrame() = default;

    CalibrationDataFrame(ExtractedTarget const& _extracted_target, Array6d const& _initial_pose,
                         ArrayX2d const& _initial_reprojection_error, Array6d const& _optimized_pose,
                         ArrayX2d const& _optimized_reprojection_error)
        : extracted_target{_extracted_target},
          initial_pose{_initial_pose},
          initial_reprojection_error{_initial_reprojection_error},
          optimized_pose{_optimized_pose},
          optimized_reprojection_error{_optimized_reprojection_error} {}

    // NOTE(Jack): We need to initialize optimized_pose with zeros otherwise we get warnings (which are considered
    // errors) about using/copying uninitialized memory (i.e. -Werror=maybe-uninitialized).
    CalibrationDataFrame(ExtractedTarget const& _extracted_target, Array6d const& _initial_pose)
        : extracted_target{_extracted_target}, initial_pose{_initial_pose}, optimized_pose{Array6d::Zero()} {}

    ExtractedTarget extracted_target;

    std::optional<Array6d> initial_pose;
    std::optional<ArrayX2d> initial_reprojection_error;

    std::optional<Array6d> optimized_pose;
    std::optional<ArrayX2d> optimized_reprojection_error;
};

using CameraFrameSequence = std::map<std::uint64_t, CalibrationDataFrame>;

struct CameraCalibrationData {
    CameraCalibrationData(CameraSensorInfo const& _sensor, ArrayXd const& _initial_intrinsics,
                          ArrayXd const& _optimized_intrinsics, CameraFrameSequence const& _frames)
        : sensor{_sensor},
          initial_intrinsics{_initial_intrinsics},
          optimized_intrinsics{_optimized_intrinsics},
          frames{_frames} {}

    explicit CameraCalibrationData(CameraSensorInfo const& _sensor) : sensor{_sensor} {}

    CameraCalibrationData(CameraSensorInfo const& _sensor, ArrayXd const& _initial_intrinsics)
        : sensor{_sensor}, initial_intrinsics{_initial_intrinsics} {}

    CameraSensorInfo sensor;

    ArrayXd initial_intrinsics;
    ArrayXd optimized_intrinsics;

    CameraFrameSequence frames;
};

}  // namespace reprojection