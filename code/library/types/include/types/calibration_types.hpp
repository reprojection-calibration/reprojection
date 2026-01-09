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
    CameraSensorInfo sensor;
    ArrayXd initial_intrinsics;
    ArrayXd optimized_intrinsics;

    CameraFrameSequence frames;
};

}  // namespace reprojection