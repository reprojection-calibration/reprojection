#pragma once

#include <map>

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// TODO(Jack): Are we sure that these types do not belong in the base types package? These types are exclusively used by
// the views for now, therefore we put them here, but I can also imagine a world where we want to use these types
// independent of the views. If that is the case than we can refactor these as view independent generic types in the
// base package.

namespace reprojection {

// TODO(Jack): Proper naming for all these types. Make about making several very similar sounding types like
// pose/frame/data etc. that are actually different! Compare with the already existing types in the types package.
struct CameraSensor {
    std::string sensor_name;
    CameraModel camera_model;
};

struct CalibrationFrame {
    ExtractedTarget extracted_target;
    Array6d initial_pose;
    Array6d optimized_pose;
};

using CalibrationData = std::map<std::uint64_t, CalibrationFrame>;

struct CameraSensorData {
    CameraSensor sensor;
    ArrayXd initial_intrinsics;
    ArrayXd optimized_intrinsics;

    CalibrationData frames;
};

}  // namespace reprojection