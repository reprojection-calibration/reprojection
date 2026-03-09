#pragma once

#include <stdexcept>
#include <string>

namespace reprojection {

// TODO(Jack): It is honestly not so nice that we need to specify the steps here and once again in the sql database, and
//  maybe once again in the python tooling. Is there any way for us to centrally store this with that repetition>
enum class CalibrationStep { Lpi, Cnlr, Sint, Snlr };

inline std::string ToString(CalibrationStep const step_name) {
    if (step_name == CalibrationStep::Lpi)
        return "linear_pose_initialization";
    else if (step_name == CalibrationStep::Cnlr) {
        return "camera_nonlinear_refinement";
    } else if (step_name == CalibrationStep::Sint) {
        return "spline_interpolation";
    } else if (step_name == CalibrationStep::Snlr) {
        return "spline_nonlinear_refinement";
    } else {
        throw std::runtime_error("Unrecognized argument passed to ToString(CalibrationStep)");  // LCOV_EXCL_LINE
    }
}

// ERROR(Jack): The intrinsic size information is currently coded in two separate places! Once here in the enum and one
// again in each projection class! This will be a pain point going forward if we do not find a solution here to
// eliminate the manual and far apart duplication.
enum class CameraModel {
    DoubleSphere = 6,  //
    Pinhole = 4,
    PinholeRadtan4 = 8,
    UnifiedCameraModel = 5,
};

// TODO(Jack): Is this the right place to put functions like this? What about testing?
inline std::string ToString(CameraModel const camera_model) {
    if (camera_model == CameraModel::Pinhole) {
        return "pinhole";
    } else if (camera_model == CameraModel::DoubleSphere) {
        return "double_sphere";
    } else {
        throw std::runtime_error("Unrecognized argument passed to ToString(CameraModel)");  // LCOV_EXCL_LINE
    }
}

inline CameraModel ToCameraModel(std::string_view camera_model) {
    if (camera_model == "pinhole") {
        return CameraModel::Pinhole;
    } else {
        throw std::runtime_error("Unrecognized argument passed to ToCameraModel(): " +  // LCOV_EXCL_LINE
                                 std::string(camera_model));                            // LCOV_EXCL_LINE
    }
}

enum class TargetType {
    Checkerboard,
    CircleGrid,
    AprilGrid3,
};

// TODO(Jack): Is this the right place to put functions like this? What about testing?
inline TargetType ToTargetType(std::string const& enum_string) {
    if (enum_string == "checkerboard") {
        return TargetType::Checkerboard;
    } else if (enum_string == "circle_grid") {
        return TargetType::CircleGrid;
    } else if (enum_string == "april_grid3") {
        return TargetType::AprilGrid3;
    } else {
        throw std::runtime_error("Unrecognized argument passed to ToTargetType(): " + enum_string);  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection