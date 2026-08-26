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

// TODO(Jack): An "asset group" is a unique and still unfinished idea within the calibration database idea. The basic
// motivation came from us needing to disambiguate steps and workflows among themselves. In any given database we might
// want multiple different workflows of the same type (i.e.  mono cam or stereo cam-cam or cam-imu calibration) but for
// different sets of sensors. If we only typed the workflows based on their type then we would only be allowed one of
// each workflow type - not good! For steps we also want to be able to have multiple types of each step within each
// workflow. For example in a stereo calibration we will have at least two image loading steps, where each step is
// itself unique based on its cache key, but is only unique within the workflow based on its asset group. I considered
// using the cache key for workflow level step disambiguation also, but I couldn't convince myself it was a good idea.
//
// When a calibration is initialized from a configuration file using steps::InitializeCalibration() it is that functions
// responsibility to correctly initialize all the asset groups, because before any workflow or workflow step can be
// created it needs to be able to satisfy a foreign constraint on an asset group. To understand how an asset group
// signature is written please see the database::AssetGroupSignature() function.
//
// Why is it unfinished? As of now (26.08.2026) there are several things I do not like about it:
//      1) The asset group is entirely specified by application logic. There is no database constraints that enforce the
//         asset group id is made up of actually existing assets, or its format in general. I understand some things the
//         application code needs to control, but the lack of database constraints on the topic scares me.
//      2) Assets groups can only be added - if they are unused or somehow changed this is not possible to represent. We
//         should remove any asset group which is not used by at least one workflow step.
//      3) Somehow the asset group id and the step cache key feel a tiny bit like they are duplicating information. Both
//         are use to disambiguate steps, with the asset group id more focused on the high level workflow semantics, and
//         the step cache key more at the low level algorithm and data, but it still seems like maybe there is some
//         possibility to reduce some redundancy, but I am not really sure that it is possible or even needed.
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