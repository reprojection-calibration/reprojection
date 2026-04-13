#pragma once

#include <optional>
#include <string>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

// TODO(Jack): These are all read operations which means I would like to be able to user SqlitePtrConst instead of the
// mutable SqlitePtr. But at this point the semantics around the sql statement lock do not allow it.
std::optional<CameraInfo> ReadCameraInfo(SqlitePtr const& db, std::string_view sensor_name);

CameraMeasurements GetExtractedTargetData(SqlitePtr const& db, std::string_view sensor_name);

std::optional<ArrayXd> ReadCameraState(SqlitePtr const& db, CalibrationStep const step_name,
                                       std::string_view sensor_name, CameraModel const camera_model);

std::optional<std::string> ReadCacheKey(SqlitePtr const& db, CalibrationStep const step_name,
                                        std::string_view sensor_name);

Frames ReadPoses(SqlitePtr const& db, CalibrationStep const step_name, std::string_view sensor_name);

ImuMeasurements GetImuData(SqlitePtr const& db, std::string_view sensor_name);

}  // namespace reprojection::database