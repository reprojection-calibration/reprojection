#pragma once

#include <memory>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::database {

using DbPtr = std::shared_ptr<CalibrationDatabase>;

void RemoveFromDb(CalibrationStep const step, std::string_view sensor_name, DbPtr const db);

}  // namespace reprojection::database