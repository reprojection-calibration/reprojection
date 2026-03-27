#pragma once

#include <stdexcept>
#include <string>

namespace reprojection {

// NOTE(Jack): We turn off code coverage for all the conversions functions because it just does not bring us much here
// If this becomes a problem we can add unit tests for the conversion functions as needed.

// LCOV_EXCL_START

// TODO(Jack): It is honestly not so nice that we need to specify the steps here and once again in the sql database, and
//  maybe once again in the python tooling. Is there any way for us to centrally store this with that repetition>
enum class CalibrationStep { CameraInfo, Cnlr, FtEx, Ii, Lpi, Sint, Snlr };

inline std::string ToString(CalibrationStep const step_name) {
    if (step_name == CalibrationStep::CameraInfo) {
        return "camera_info";
    } else if (step_name == CalibrationStep::Cnlr) {
        return "camera_nonlinear_refinement";
    } else if (step_name == CalibrationStep::FtEx) {
        return "feature_extraction";
    } else if (step_name == CalibrationStep::Ii) {
        return "intrinsic_initialization";
    } else if (step_name == CalibrationStep::Lpi) {
        return "linear_pose_initialization";
    } else if (step_name == CalibrationStep::Sint) {
        return "spline_interpolation";
    } else if (step_name == CalibrationStep::Snlr) {
        return "spline_nonlinear_refinement";
    } else {
        throw std::runtime_error(
            "LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToString(CalibrationStep)");
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
    if (camera_model == CameraModel::DoubleSphere) {
        return "double_sphere";
    } else if (camera_model == CameraModel::Pinhole) {
        return "pinhole";
    } else if (camera_model == CameraModel::PinholeRadtan4) {
        return "pinhole_radtan4";
    } else if (camera_model == CameraModel::UnifiedCameraModel) {
        return "unified_camera_model";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR -Unrecognized argument passed to ToString(CameraModel)");
    }
}

inline CameraModel ToCameraModel(std::string_view camera_model) {
    if (camera_model == "double_sphere") {
        return CameraModel::DoubleSphere;
    } else if (camera_model == "pinhole") {
        return CameraModel::Pinhole;
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToCameraModel(): " +
                                 std::string(camera_model));
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
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToTargetType(): " +
                                 enum_string);
    }
}

enum class InitializationType {
    ParabolaLine,
    VanishingPoint,
};

enum class CacheStatus {
    CacheHit,
    CacheMiss,
};

// LCOV_EXCL_START
inline std::string ToString(CacheStatus const status) {
    if (status == CacheStatus::CacheHit) {
        return "cache_hit";
    } else if (status == CacheStatus::CacheMiss) {
        return "cache_miss";
    } else {
        throw std::runtime_error{"LIBRARY IMPLEMENTATION ERROR - ToString(CacheStatus)"};
    }
}
// LCOV_EXCL_STOP

}  // namespace reprojection