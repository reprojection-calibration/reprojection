#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

// TODO(Jack): See the note below for InsertExtrinsic(). Same problems are plaguing us here.
void InsertGravity(Array3d const& data, CalibrationStep const step_name, std::string_view sensor_name,
                   SqlitePtr const db);

// TODO(Jack): We are hardcoding into the database that there is only ever on extrinsic - from one camera to one imu.
// This will not scale if we add multiple cameras or multiple IMUs. But instead of overengineering from the start I am
// going to wait until we need multisensor extrinsics instead of trying to guess what we will need.
// NOTE(Jack): Because it is so opaque what is getting written to the database (it is just a random Array6d )
void InsertExtrinsic(Array6d const& data, CalibrationStep const step_name, std::string_view sensor_name,
                     SqlitePtr const db);

// NOTE(Jack): The calibration step has "upsert" semantics (https://sqlite.org/lang_upsert.html) because we need to
// update the cache_key when the steps update on reruns.
void InsertStep(CalibrationStep const step_name, std::optional<std::string_view> cache_key, std::string_view sensor_name,
               SqlitePtr const db);

void InsertCameraInfo(CameraInfo const& camera_info, SqlitePtr const db);

void InsertTargets(CameraMeasurements const& data, std::string_view sensor_name, SqlitePtr const db);

void InsertIntrinsics(CameraState const& data, CameraModel const camera_model, CalibrationStep const step_name,
               std::string_view sensor_name, SqlitePtr const db);

void InsertImages(EncodedImages const& data, std::string_view sensor_name, SqlitePtr const db);

void InsertPoses(Frames const& data, CalibrationStep const step_name, std::string_view sensor_name, SqlitePtr const db);

void InsertImuErrors(ImuErrors const& data, CalibrationStep const step_name, std::string_view sensor_name,
               SqlitePtr const db);

void InsertImuData(ImuMeasurements const& data, std::string_view sensor_name, SqlitePtr const db);

void WriteToDb(ReprojectionErrors const& data, CalibrationStep const step_name, std::string_view sensor_name,
               SqlitePtr const db);

// WARN(Jack): This is a hack! There is no requirement for a target to have a sensor name! This should get removed one
// day when we successfully abstract the pipeline to handle multi-target calibration.
void WriteToDb(TargetInfo const& target_info, std::string_view sensor_name, SqlitePtr const db);

// WARN(Jack): Hardcoded for an SE3 spline. Nx6 control point block.
void WriteToDb(spline::Matrix2NXd const& data, CalibrationStep const step_name, std::string_view sensor_name,
               SqlitePtr const db);

void WriteToDb(spline::TimeHandler const& data, CalibrationStep const step_name, std::string_view sensor_name,
               SqlitePtr const db);

}  // namespace reprojection::database