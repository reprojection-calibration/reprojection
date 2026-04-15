#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

// TODO(Jack): Add note in docs that we are hardcoding one target only by not adding a target_id identifier. If we want
//  multitarget setups things are gonna be more complicated all around.
// TODO(Jack): Add not in docs that we cannot cover all error conditions in unit test so we suppress the errors

namespace reprojection::database {

void WriteToDb(CameraInfo const& camera_info, SqlitePtr const db);

// NOTE(Jack): The calibration step has "upsert" semantics (https://sqlite.org/lang_upsert.html) because we need to
// update the cache_key when the steps update on reruns.
void WriteToDb(CalibrationStep const step_name, std::optional<std::string_view> cache_key, std::string_view sensor_name,
               SqlitePtr const db);

void WriteToDb(CameraMeasurements const& data, std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(CameraState const& data, CameraModel const camera_model, CalibrationStep const step_name,
               std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(Frames const& data, CalibrationStep const step_name, std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(ReprojectionErrors const& data, CalibrationStep const step_name, std::string_view sensor_name,
               SqlitePtr const db);

void WriteToDb(ImuMeasurements const& data, std::string_view sensor_name, SqlitePtr const db);

}  // namespace reprojection::database