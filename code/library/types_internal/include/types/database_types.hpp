#pragma once

#include <cstdint>
#include <string>

namespace reprojection {

struct AssetId {
    int64_t value;

    friend constexpr bool operator==(AssetId const&, AssetId const&) = default;
};

struct RecordingId {
    int64_t value;

    friend constexpr bool operator==(RecordingId const&, RecordingId const&) = default;
};

struct RunId {
    int64_t value;

    friend constexpr bool operator==(RunId const&, RunId const&) = default;
};

struct StepId {
    int64_t value;

    friend constexpr bool operator==(StepId const&, StepId const&) = default;
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
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unknown AssetType");
    }
}

enum class StepType {
    FeatureExtraction,
    ImageLoading,
    ImuDataLoading,
    TargetInfo,
};

inline std::string ToString(StepType const data) {
    if (data == StepType::FeatureExtraction) {
        return "feature_extraction";
    } else if (data == StepType::ImageLoading) {
        return "image_loading";
    } else if (data == StepType::ImuDataLoading) {
        return "imu_data_loading";
    } else if (data == StepType::TargetInfo) {
        return "target_info";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unknown StepType");
    }
}

}  // namespace reprojection