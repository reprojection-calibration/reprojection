#pragma once

#include <memory>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

CameraMeasurements GetExtractedTargetData(std::shared_ptr<CalibrationDatabase const> const database,
                                          std::string_view sensor_name);

// TODO(Jack): Will one day associate each cache key in its own tables which also specifies the sensor_name.
std::optional<std::string> GetCacheKey(std::shared_ptr<CalibrationDatabase const> const database,
                                       std::string_view step_name);

std::optional<std::pair<CameraInfo, CameraState>> GetIntrinsic(
    std::shared_ptr<CalibrationDatabase const> const database, std::string_view step_name,
    std::string_view sensor_name);

ImuMeasurements GetImuData(std::shared_ptr<CalibrationDatabase const> const database, std::string_view sensor_name);

}  // namespace reprojection::database