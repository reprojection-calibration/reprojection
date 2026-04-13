#pragma once

#include <memory>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::database {

void RemoveFromDb(CalibrationStep const step, std::string_view sensor_name, SqlitePtr const& db);

}  // namespace reprojection::database