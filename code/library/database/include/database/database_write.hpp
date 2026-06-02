#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

// NOTE(Jack): The calibration step has "upsert" semantics (https://sqlite.org/lang_upsert.html) because we need to
// update the cache_key when the steps update on reruns.
void WriteToDb(CalibrationStep const step_name, std::optional<std::string_view> cache_key, std::string_view sensor_name,
               SqlitePtr const db);

void WriteToDb(CameraInfo const& camera_info, SqlitePtr const db);

void WriteToDb(CameraMeasurements const& data, std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(CameraState const& data, CameraModel const camera_model, CalibrationStep const step_name,
               std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(EncodedImages const& data, std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(Frames const& data, CalibrationStep const step_name, std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(ImuMeasurements const& data, std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(ReprojectionErrors const& data, CalibrationStep const step_name, std::string_view sensor_name,
               SqlitePtr const db);

// WARN(Jack): This is a hack! There is no requirement for a target to have a sensor name! This should get removed one
// day when we successfully abstract the pipeline to handle multi-target calibration.
void WriteToDb(TargetInfo const& target_info, std::string_view sensor_name, SqlitePtr const db);

// WARN(Jack): Hardcoded for an SE3 spline. Nx6 control point block.
void WriteToDb(spline::Matrix2NXd const& data, CalibrationStep const step_name, std::string_view sensor_name,
               SqlitePtr const db);

}  // namespace reprojection::database