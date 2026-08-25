#pragma once

#include <expected>
#include <filesystem>

#include "spline/time_handler.hpp"
#include "spline/types.hpp"
#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

SqlitePtr OpenCalibrationDatabase(std::filesystem::path const& db_path, bool create, bool read_only = false);

void AssetGroupInsert(sqlite3* db, std::vector<AssetId> const& asset_ids);

AssetId GetOrCreateAsset(sqlite3* db, AssetType type, size_t index, Name const& name);

WorkflowId GetOrCreateWorkflow(sqlite3* db, WorkflowType type, std::vector<AssetId> const& assets);

void WorkflowAssetsInsert(sqlite3* db, WorkflowId workflow_id, std::vector<AssetId> const& asset_ids);

void WorkflowStepUpsert(sqlite3* db, WorkflowId workflow_id, StepId step_id, StepType step_type,
                        std::vector<AssetId> const& asset_ids);

// TODO(Jack): The semantics of this step method are so different from the others that we should probably not use
// the same name. bool: was this a cache hit?
std::pair<StepId, CacheStatus> GetOrCreateStep(sqlite3* db, StepType type, Hash const& cache_key);

// NOTE(Jack): We need the step creation and cache key insertion to be separate because if the step execution fails
// we do not want stale/bad cache keys in the database. By splitting this up and implementing it carefully in the
// step running logic we can ensure a cache key only gets written if the execution was succesful.
void StepCacheKeyUpdate(sqlite3* db, StepId step_id, Hash const& cache_key);

void CameraInfoInsert(sqlite3* db, StepId step_id, AssetId asset_id, CameraInfo const& camera_info);

std::expected<CameraInfo, std::string> CameraInfoSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void CameraPosesInsert(sqlite3* db, StepId step_id, StepId source_step_id, AssetId asset_id,
                       Frames const& camera_poses);

Frames CameraPosesSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ControlPointsInsert(sqlite3* db, StepId step_id, AssetId asset_id, spline::Matrix2NXd const& data);

spline::Matrix2NXd ControlPointsSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ExtractedTargetsInsert(sqlite3* db, StepId step_id, StepId source_step_id, AssetId asset_id,
                            CameraMeasurements const& data);

CameraMeasurements ExtractedTargetsSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ExtrinsicInsert(sqlite3* db, StepId step_id, Extrinsic const& extrinsic);

std::expected<Extrinsic, std::string> ExtrinsicSelect(sqlite3* db, StepId step_id, AssetId asset_a_id,
                                                      AssetId asset_b_id);

void GravityInsert(sqlite3* db, StepId step_id, Vector3d const& gravity);

std::expected<Vector3d, std::string> GravitySelect(sqlite3* db, StepId step_id);

void ImagesInsert(sqlite3* db, StepId step_id, AssetId asset_id, EncodedImages const& data);

EncodedImages ImagesSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ImuDataInsert(sqlite3* db, StepId step_id, AssetId asset_id, ImuMeasurements const& data);

ImuMeasurements ImuDataSelect(sqlite3* db, StepId step_id, AssetId asset_id);

// TODO(Jack): "source_step_id" used here and elsewhere is a misleading name. In reality this step id is used only to
// establish a foreign key constraint, there is nothing to do with the source of anything (at least that is not a
// requirement). We need to think of a better name/concept I think.
void ImuErrorsInsert(sqlite3* db, StepId step_id, StepId source_step_id, AssetId asset_id, ImuErrors const& data);

void IntrinsicInsert(sqlite3* db, StepId step_id, AssetId asset_id, CameraModel camera_model, CameraState const& data);

// TODO(Jack): Should this also return the camera_model? We have that information.
std::expected<CameraState, std::string> IntrinsicSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void ReprojectionErrorsInsert(sqlite3* db, StepId step_id, StepId source_step_id, AssetId asset_id,
                              ReprojectionErrors const& data);

void SplineInfoInsert(sqlite3* db, StepId step_id, AssetId asset_id, spline::TimeHandler const& time_handler);

std::expected<spline::TimeHandler, std::string> SplineInfoSelect(sqlite3* db, StepId step_id, AssetId asset_id);

void TargetInfoInsert(sqlite3* db, StepId step_id, AssetId asset_id, TargetInfo const& target_info);

std::expected<TargetInfo, std::string> TargetInfoSelect(sqlite3* db, StepId step_id, AssetId asset_id);

}  // namespace reprojection::database