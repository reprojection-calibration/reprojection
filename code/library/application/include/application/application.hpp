#pragma once
#include <memory>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::application {

enum class CacheStatus {
    CacheHit,
    CacheKeyMiss,
    StepNameMiss,
};

std::pair<CacheStatus, CameraState> FocalLengthInitialization(
    CameraMeasurements const& targets, CameraInfo const& camera_info,
    std::shared_ptr<database::CalibrationDatabase> const database);

CacheStatus CacheState(std::shared_ptr<database::CalibrationDatabase const> const database, std::string_view step_name,
                       std::string_view cache_key);

}  // namespace reprojection::application