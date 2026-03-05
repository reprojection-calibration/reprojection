#pragma once

#include <memory>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::application {

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
                       std::string_view key);

}  // namespace reprojection::application