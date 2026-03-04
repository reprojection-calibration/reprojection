#pragma once
#include <memory>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::application {

// TODO MOVE TO TYPES
enum class CacheStatus {
    CacheHit,
    CacheKeyMiss,
    StepNameMiss,
};

inline std::string ToString(CacheStatus const cache_status) {
    if (cache_status == CacheStatus::CacheHit) {
        return "cache_hit";
    } else if (cache_status == CacheStatus::CacheKeyMiss) {
        return "cache_key_miss";
    } else if (cache_status == CacheStatus::StepNameMiss) {
        return "step_name_miss";
    } else {
        throw std::runtime_error("Library implementation error ToString(CacheStatus)");
    }
}

std::pair<CacheStatus, CameraState> FocalLengthInitialization(
    CameraMeasurements const& targets, CameraInfo const& camera_info,
    std::shared_ptr<database::CalibrationDatabase> const database);

std::pair<CacheStatus, OptimizationState> LinearPoseInitialization(
    CameraInfo const& camera_info, CameraMeasurements const& targets, CameraState const& camera_state,
    std::shared_ptr<database::CalibrationDatabase> const database);

std::pair<CacheStatus, OptimizationState> CameraNonlinearRefinement(
    CameraInfo const& camera_info, CameraMeasurements const& targets, OptimizationState const& optimization_state,
    std::shared_ptr<database::CalibrationDatabase> const database);


CacheStatus CacheState(std::shared_ptr<database::CalibrationDatabase const> const database, std::string_view step_name,
                       std::string_view cache_key);

}  // namespace reprojection::application