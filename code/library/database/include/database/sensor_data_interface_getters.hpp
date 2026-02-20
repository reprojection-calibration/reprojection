#pragma once

#include <memory>

#include <opencv2/opencv.hpp>

#include "database/calibration_database.hpp"
#include "database/database_data_types.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_types.hpp"

namespace reprojection::database {

CameraMeasurements GetExtractedTargetData(std::shared_ptr<CalibrationDatabase const> const database,
                                          std::string_view sensor_name);

ImuMeasurements GetImuData(std::shared_ptr<CalibrationDatabase const> const database, std::string_view sensor_name);

}  // namespace reprojection::database