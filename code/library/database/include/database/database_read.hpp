#pragma once

#include <optional>
#include <string>

#include "database/calibration_database.hpp"
#include "spline/time_handler.hpp"
#include "spline/types.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

std::optional<std::string> ReadCacheKey(SqlitePtr const db, CalibrationStep const step_name,
                                        std::string_view sensor_name);

// TODO(Jack): These are all read operations which means I would like to be able to user SqlitePtrConst instead of the
// mutable SqlitePtr. But at this point the semantics around the sql statement lock do not allow it.
std::optional<CameraInfo> ReadCameraInfo(SqlitePtr const db, std::string_view sensor_name);

std::optional<ArrayXd> ReadCameraState(SqlitePtr const db, CalibrationStep const step_name,
                                       std::string_view sensor_name, CameraModel const camera_model);

EncodedImages ReadEncodedImages(SqlitePtr const db, std::string_view sensor_name);

CameraMeasurements ReadExtractedTargets(SqlitePtr const db, std::string_view sensor_name);

ImuMeasurements ReadImuData(SqlitePtr const db, std::string_view sensor_name);

Frames ReadPoses(SqlitePtr const db, CalibrationStep const step_name, std::string_view sensor_name);

// TODO(Jack): See note in database_write.hpp how having the sensor name here is a hack and should be removed one day!
std::optional<TargetInfo> ReadTargetInfo(SqlitePtr const db, std::string_view sensor_name);

spline::Matrix2NXd ReadSplineControlPoints(SqlitePtr const db, CalibrationStep const step_name,
                                           std::string_view sensor_name);

std::optional<spline::TimeHandler> ReadSplineTimeHandler(SqlitePtr const db, CalibrationStep const step_name,
                                                         std::string_view sensor_name);

}  // namespace reprojection::database