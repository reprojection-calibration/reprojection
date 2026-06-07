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

std::optional<std::string> ReadCacheKey(SqlitePtr const db, std::string_view sensor_name,
                                        CalibrationStep const step_name);

// TODO(Jack): These are all read operations which means I would like to be able to user SqlitePtrConst instead of the
// mutable SqlitePtr. But at this point the semantics around the sql statement lock do not allow it.
std::optional<CameraInfo> ReadCameraInfo(SqlitePtr const db, std::string_view sensor_name);

std::optional<ArrayXd> ReadIntrinsics(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                                      CameraModel const camera_model);

EncodedImages ReadImages(SqlitePtr const db, std::string_view sensor_name);

CameraMeasurements ReadTargets(SqlitePtr const db, std::string_view sensor_name);

// TODO(Jack): One day we need to redesign this with a more informative extrinsic calibration type that also describes
// which frames it converts between. Right now this is pretty rudimentary to just load an array here and hijack the
// sensor_name field to hold something like tf_imu_co. Hacky!
std::optional<Extrinsic> ReadExtrinsics(SqlitePtr const db, std::string_view sensor_name,
                                        CalibrationStep const step_name);

std::optional<Array3d> ReadGravity(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name);

ImuMeasurements ReadImuData(SqlitePtr const db, std::string_view sensor_name);

ImuErrors ReadImuErrors(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name);

Frames ReadPoses(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name);

// TODO(Jack): See note in database_write.hpp how having the sensor name here is a hack and should be removed one day!
std::optional<TargetInfo> ReadTargetInfo(SqlitePtr const db, std::string_view sensor_name);

spline::Matrix2NXd ReadControlPoints(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name);

std::optional<spline::TimeHandler> ReadTimeHandler(SqlitePtr const db, std::string_view sensor_name,
                                                   CalibrationStep const step_name);

}  // namespace reprojection::database