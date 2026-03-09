#pragma once

#include <memory>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

using DbConstPtr = std::shared_ptr<CalibrationDatabase const>;

std::optional<CameraInfo> ReadCameraInfo(DbConstPtr const database, std::string_view sensor_name);

CameraMeasurements GetExtractedTargetData(DbConstPtr const database, std::string_view sensor_name);

std::optional<ArrayXd> ReadCameraState(DbConstPtr const database, CalibrationStep const step_name,
                                       std::string_view sensor_name, CameraModel const camera_model);

std::optional<std::string> ReadCacheKey(DbConstPtr const database, CalibrationStep const step_name,
                                        std::string_view sensor_name);

Frames ReadPoses(DbConstPtr const database, CalibrationStep const step_name, std::string_view sensor_name);

ImuMeasurements GetImuData(DbConstPtr const database, std::string_view sensor_name);

}  // namespace reprojection::database