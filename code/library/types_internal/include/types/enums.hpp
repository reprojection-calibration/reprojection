#pragma once

#include <stdexcept>
#include <string>

namespace reprojection {

// NOTE(Jack): We turn off code coverage for all the conversions functions because it just does not bring us much here
// If this becomes a problem we can add unit tests for the conversion functions as needed.

// LCOV_EXCL_START

enum class Entity { Camera, Extrinsic, Imu, Target };

inline std::string ToString(Entity const entity_id) {
    if (entity_id == Entity::Camera) {
        return "camera";
    } else if (entity_id == Entity::Extrinsic) {
        return "extrinsic";
    } else if (entity_id == Entity::Imu) {
        return "imu";
    } else if (entity_id == Entity::Target) {
        return "target";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToString(Entity)");
    }
}

// WARN(Jack): If tou add a step here but forget to add it to the ToString() function below you will get really cryptic
// errors from the sql database because it does no rethrow the error properly.
// TODO(Jack): It is honestly not so nice that we need to specify the steps here and once again in the sql database, and
// maybe once again in the python tooling. Is there any way for us to centrally store this with that repetition.
enum class CalibrationStep {
    BundleAdjustment,
    CameraInfo,
    ExtrinsicInitialization,
    FeatureExtraction,
    ImageLoading,
    IntrinsicInitialization,
    PoseInitialization,
    SplineInitialization,
    TargetInfo
};

inline std::string ToString(CalibrationStep const step_name) {
    if (step_name == CalibrationStep::BundleAdjustment) {
        return "bundle_adjustment";
    } else if (step_name == CalibrationStep::CameraInfo) {
        return "camera_info";
    } else if (step_name == CalibrationStep::ExtrinsicInitialization) {
        return "extrinsic_initialization";
    } else if (step_name == CalibrationStep::FeatureExtraction) {
        return "feature_extraction";
    } else if (step_name == CalibrationStep::ImageLoading) {
        return "image_loading";
    } else if (step_name == CalibrationStep::IntrinsicInitialization) {
        return "intrinsic_initialization";
    } else if (step_name == CalibrationStep::PoseInitialization) {
        return "pose_initialization";
    } else if (step_name == CalibrationStep::SplineInitialization) {
        return "spline_initialization";
    } else if (step_name == CalibrationStep::TargetInfo) {
        return "target_info";
    } else {
        throw std::runtime_error(
            "LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToString(CalibrationStep)");
    }
}

// ERROR(Jack): The intrinsic size information is currently coded in two separate places! Once here in the enum and one
// again in each projection class! This will be a pain point going forward if we do not find a solution here to
// eliminate the manual and far apart duplication.
enum class CameraModel {
    DoubleSphere = 5,  //
    Pinhole = 3,
    PinholeRadtan4 = 7,
    UnifiedCameraModel = 4,
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
    } else if (camera_model == "pinhole_radtan4") {
        return CameraModel::PinholeRadtan4;
    } else if (camera_model == "unified_camera_model") {
        return CameraModel::UnifiedCameraModel;
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToCameraModel(): " +
                                 std::string(camera_model));
    }
}

enum class TargetType {
    Checkerboard,
    CircleGrid,
    Aprilgrid3,
};

inline std::string ToString(TargetType const target_type) {
    if (target_type == TargetType::Aprilgrid3) {
        return "aprilgrid3";
    } else if (target_type == TargetType::Checkerboard) {
        return "checkerboard";
    } else if (target_type == TargetType::CircleGrid) {
        return "circle_grid";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unrecognized argument passed to ToString(TargetType)");
    }
}

// TODO(Jack): Is this the right place to put functions like this? What about testing?
inline TargetType ToTargetType(std::string const& enum_string) {
    if (enum_string == "checkerboard") {
        return TargetType::Checkerboard;
    } else if (enum_string == "circle_grid") {
        return TargetType::CircleGrid;
    } else if (enum_string == "aprilgrid3") {
        return TargetType::Aprilgrid3;
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