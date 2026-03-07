#pragma once

#include <memory>

#include "database/calibration_database.hpp"
#include "types/enums.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

CameraMeasurements GetExtractedTargetData(std::shared_ptr<CalibrationDatabase const> const database,
                                          std::string_view sensor_name);

std::optional<std::string> ReadCacheKey(std::shared_ptr<CalibrationDatabase const> const database,
                                        CalibrationStep const step_name, std::string_view sensor_name);

ImuMeasurements GetImuData(std::shared_ptr<CalibrationDatabase const> const database, std::string_view sensor_name);

}  // namespace reprojection::database