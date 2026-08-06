#pragma once

#include <filesystem>
#include <optional>

#include "spline/time_handler.hpp"
#include "spline/types.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {



SqlitePtr OpenCalibrationDatabase(std::filesystem::path const& db_path, bool create, bool read_only = false);

AssetId GetOrCreateAsset(sqlite3* db, AssetType type, size_t index, Name const& name);

WorkflowId GetOrCreateWorkflow(sqlite3* db, WorkflowType type, std::vector<AssetId> const& assets);

// TODO(Jack): The semantics of this step method are so different from the others that we should probably not use
// the same name. bool: was this a cache hit?
std::pair<StepId, CacheStatus> GetOrCreateStep(sqlite3* db, StepType type, Hash const& cache_key);

// NOTE(Jack): We need the step creation and cache key insertion to be separate because if the step execution fails
// we do not want stale/bad cache keys in the database. By splitting this up and implementing it carefully in the
// step running logic we can ensure a cache key only gets written if the execution was succesful.
void StepCacheKeyUpdate(sqlite3* db, StepId step_id, Hash const& cache_key);

void CameraInfoInsert(sqlite3* db, StepId step_id, AssetId asset_id, CameraInfo const& camera_info);

std::optional<CameraInfo> CameraInfoSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void CameraPosesInsert(sqlite3* db, StepId step_id, StepId source_step_id, AssetId asset_id,
                       Frames const& camera_poses);

Frames CameraPosesSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ControlPointsInsert(sqlite3* db, StepId step_id, AssetId asset_id, spline::Matrix2NXd const& data);

spline::Matrix2NXd ControlPointsSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ExtrinsicInsert(sqlite3* db, StepId step_id, Extrinsic const& extrinsic);

std::optional<Extrinsic> ExtrinsicSelect(sqlite3* db, StepId step_id, AssetId asset_a_id, AssetId asset_b_id);

void GravityInsert(sqlite3* db, StepId step_id, Vector3d const& gravity);

std::optional<Vector3d> GravitySelect(sqlite3* db, StepId step_id);

void ImagesInsert(sqlite3* db, StepId step_id, AssetId asset_id, EncodedImages const& data);

EncodedImages ImagesSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ImuDataInsert(sqlite3* db, StepId step_id, AssetId asset_id, ImuMeasurements const& data);

ImuMeasurements ImuDataSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void IntrinsicInsert(sqlite3* db, StepId step_id, AssetId asset_id, CameraModel camera_model, CameraState const& data);

// TODO(Jack): Should this also return the camera_model? We have that information.
std::optional<CameraState> IntrinsicSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ExtractedTargetsInsert(sqlite3* db, StepId step_id, StepId source_step_id, AssetId asset_id,
                            CameraMeasurements const& data);

CameraMeasurements ExtractedTargetsSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void SplineInfoInsert(sqlite3* db, StepId step_id, AssetId asset_id, spline::TimeHandler const& time_handler);

std::optional<spline::TimeHandler> SplineInfoSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void TargetInfoInsert(sqlite3* db, StepId step_id, AssetId asset_id, TargetInfo const& target_info);

std::optional<TargetInfo> TargetInfoSelect(sqlite3* db, StepId step_id, AssetId asset_id);

}  // namespace reprojection::database