#pragma once

#include <stdexcept>
#include <string>

namespace reprojection {

struct AssetId {
    int64_t value;

    friend constexpr bool operator==(AssetId const&, AssetId const&) = default;
};

struct WorkflowId {
    int64_t value;

    friend constexpr bool operator==(WorkflowId const&, WorkflowId const&) = default;
};

struct StepId {
    int64_t value;

    friend constexpr bool operator==(StepId const&, StepId const&) = default;
};

struct Hash {
    // NOTE(Jack): It is just way easier to let this auto convert that we allow the Name and Hash constructors to not be
    // explicit.
    //
    // cppcheck-suppress noExplicitConstructor
    Hash(std::string_view _value) : value{_value} {}

    // cppcheck-suppress noExplicitConstructor
    Hash(char const* const value) : Hash{std::string_view{value}} {}

    Hash() = default;

    std::string value;

    friend constexpr bool operator==(Hash const&, Hash const&) = default;
};

struct Name {
    // cppcheck-suppress noExplicitConstructor
    Name(std::string const& _value) : value{_value} {}

    // cppcheck-suppress noExplicitConstructor
    Name(std::string_view _value) : value{_value} {}

    // cppcheck-suppress noExplicitConstructor
    Name(char const* const value) : Name{std::string_view{value}} {}

    std::string value;

    friend constexpr bool operator==(Name const&, Name const&) = default;
};

enum class AssetType { Camera, Imu, Target };

inline std::string ToString(AssetType const data) {
    if (data == AssetType::Camera) {
        return "camera";
    } else if (data == AssetType::Imu) {
        return "imu";
    } else if (data == AssetType::Target) {
        return "target";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unknown AssetType");  // LCOV_EXCL_LINE
    }
}

enum class WorkflowType { Cam, CamImu, MultiCam };

inline std::string ToString(WorkflowType const data) {
    if (data == WorkflowType::Cam) {
        return "cam";
    } else if (data == WorkflowType::CamImu) {
        return "cam_imu";
    } else if (data == WorkflowType::MultiCam) {
        return "multi_cam";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unknown WorkflowType");  // LCOV_EXCL_LINE
    }
}

enum class StepType {
    BundleAdjustment,
    CameraInfo,
    ExtrinsicInit,
    ExtrinsicOptimization,
    FeatureExtraction,
    ImageLoading,
    ImuDataLoading,
    IntrinsicInit,
    PoseInit,
    SplineInit,
    TargetInfo,
};

inline std::string ToString(StepType const data) {
    if (data == StepType::BundleAdjustment) {
        return "bundle_adjustment";
    } else if (data == StepType::CameraInfo) {
        return "camera_info";
    } else if (data == StepType::ExtrinsicInit) {
        return "extrinsic_initialization";
    } else if (data == StepType::ExtrinsicOptimization) {
        return "extrinsic_optimization";  // LCOV_EXCL_LINE
    } else if (data == StepType::FeatureExtraction) {
        return "feature_extraction";
    } else if (data == StepType::ImageLoading) {
        return "image_loading";
    } else if (data == StepType::ImuDataLoading) {
        return "imu_data_loading";
    } else if (data == StepType::IntrinsicInit) {
        return "intrinsic_initialization";
    } else if (data == StepType::PoseInit) {
        return "pose_initialization";
    } else if (data == StepType::SplineInit) {
        return "spline_initialization";
    } else if (data == StepType::TargetInfo) {
        return "target_info";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unknown StepType");  // LCOV_EXCL_LINE
    }
}

// TODO(Jack): Is this really the base name for what this does? We really need a better name here.
template <typename T>
struct Asset {
    AssetId id;
    T config;
};

}  // namespace reprojection