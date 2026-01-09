#pragma once

#include <map>
#include <string>

#include "types/algorithm_types.hpp"
#include "types/camera_types.hpp"
#include "types/eigen_types.hpp"

// TODO(Jack): Make sure the names here to conflict logically with other types.

namespace reprojection {

struct CameraSensorInfo {
    std::string sensor_name;
    CameraModel camera_model;
};

struct CalibrationDataFrame {
    ExtractedTarget extracted_target;
    Array6d initial_pose;
    Array6d optimized_pose;
};

using CameraFrameSequence = std::map<std::uint64_t, CalibrationDataFrame>;

struct CameraCalibrationData {
    CameraCalibrationData(CameraSensorInfo const& _sensor, ArrayXd const& _initial_intrinsics,
                          ArrayXd const& _optimized_intrinsics, CameraFrameSequence const& _frames)
        : sensor{_sensor},
          initial_intrinsics{_initial_intrinsics},
          optimized_intrinsics{_optimized_intrinsics},
          frames{_frames} {}

    CameraCalibrationData(CameraSensorInfo const& _sensor) : sensor{_sensor} {}

    CameraCalibrationData(CameraSensorInfo const& _sensor, ArrayXd const& _initial_intrinsics)
        : sensor{_sensor}, initial_intrinsics{_initial_intrinsics} {}

    CameraSensorInfo sensor;
    ArrayXd initial_intrinsics;
    ArrayXd optimized_intrinsics;

    CameraFrameSequence frames;
};

}  // namespace reprojection