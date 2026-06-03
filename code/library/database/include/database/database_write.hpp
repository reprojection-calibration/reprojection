#pragma once

#include "database/calibration_database.hpp"
#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

// TODO(Jack): See the note below for InsertExtrinsic(). Same problems are plaguing us here.
void InsertGravity(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                   Array3d const& data);

// TODO(Jack): We are hardcoding into the database that there is only ever on extrinsic - from one camera to one imu.
// This will not scale if we add multiple cameras or multiple IMUs. But instead of overengineering from the start I am
// going to wait until we need multisensor extrinsics instead of trying to guess what we will need.
// NOTE(Jack): Because it is so opaque what is getting written to the database (it is just a random Array6d )
void InsertExtrinsic(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                     Array6d const& data);

// NOTE(Jack): The calibration step has "upsert" semantics (https://sqlite.org/lang_upsert.html) because we need to
// update the cache_key when the steps update on reruns.
void InsertStep(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                std::optional<std::string_view> cache_key);

void InsertCameraInfo(SqlitePtr const db, CameraInfo const& camera_info);

void InsertTargets(SqlitePtr const db, std::string_view sensor_name, CameraMeasurements const& data);

void InsertIntrinsics(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                      CameraModel const camera_model, CameraState const& data);

void InsertImages(SqlitePtr const db, std::string_view sensor_name, EncodedImages const& data);

void InsertPoses(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name, Frames const& data);

void InsertImuErrors(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                     ImuErrors const& data);

void InsertImuData(SqlitePtr const db, std::string_view sensor_name, ImuMeasurements const& data);

void InsertReprojectionErrors(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                              ReprojectionErrors const& data);

// WARN(Jack): This is a hack! There is no requirement for a target to have a sensor name! This should get removed one
// day when we successfully abstract the pipeline to handle multi-target calibration.
void InsertTargetInfo(SqlitePtr const db, std::string_view sensor_name, TargetInfo const& target_info);

// WARN(Jack): Hardcoded for an SE3 spline. Nx6 control point block.
void InsertControlPoints(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                         spline::Matrix2NXd const& data);

void InsertTimeHandler(SqlitePtr const db, std::string_view sensor_name, CalibrationStep const step_name,
                       spline::TimeHandler const& data);

}  // namespace reprojection::database