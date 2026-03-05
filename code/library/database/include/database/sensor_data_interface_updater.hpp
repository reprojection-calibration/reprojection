#pragma once

#include <memory>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

void UpdateCalibrationStep(std::string_view step_name, std::string_view cache_key,
                        std::shared_ptr<CalibrationDatabase> const database);


}  // namespace reprojection::database