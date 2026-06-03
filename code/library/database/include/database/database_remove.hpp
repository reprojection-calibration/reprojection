#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::database {

void RemoveFromDb(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step);

}  // namespace reprojection::database