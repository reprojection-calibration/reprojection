#pragma once

#include <sqlite3.h>

#include <filesystem>
#include <optional>

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

namespace fs = std::filesystem;

class CalibrationDatabase {
   public:
    // TODO(Jack): Should we make this private and instead use a factory?
    CalibrationDatabase(fs::path const& db_path, bool create, bool read_only = false);

    ~CalibrationDatabase();

    AssetId GetOrCreateAsset(AssetType type, size_t index, Name const& name) const;

    RecordingId GetOrCreateRecording(Name const& name, Hash const& hash) const;

    // TODO(Jack): The semantics of this step method are so different from the others that we should probably not use
    // the same name. bool: was this a cache hit?
    std::pair<StepId, CacheStatus> GetOrCreateStep(StepType type, Hash const& cache_key);

    // NOTE(Jack): We need the step creation and cache key insertion to be separate because if the step execution fails
    // we do not want stale/bad cache keys in the database. By splitting this up and implementing it carefully in the
    // step running logic we can ensure a cache key only gets written if the execution was succesful.
    void StepCacheKeyUpdate(StepId step_id, Hash const& cache_key) const;

    void CameraInfoInsert(StepId step_id, AssetId asset_id, CameraInfo const& camera_info) const;

    std::optional<CameraInfo> CameraInfoSelect(StepId step_id, AssetId asset_id) const;

    void CameraPosesInsert(StepId step_id, StepId source_step_id, AssetId asset_id, Frames const& camera_poses) const;

    Frames CameraPosesSelect(StepId step_id, AssetId asset_id) const;

    void ImagesInsert(StepId step_id, AssetId asset_id, EncodedImages const& data) const;

    EncodedImages ImagesSelect(StepId step_id, AssetId asset_id) const;

    void ImuDataInsert(StepId step_id, AssetId asset_id, ImuMeasurements const& data) const;

    ImuMeasurements ImuDataSelect(StepId step_id, AssetId asset_id) const;

    void IntrinsicInsert(StepId step_id, AssetId asset_id, CameraModel camera_model, CameraState const& data) const;

    // TODO(Jack): Should this also return the camera_model? We have that information.
    std::optional<CameraState> IntrinsicSelect(StepId step_id, AssetId asset_id) const;

    void ExtractedTargetsInsert(StepId step_id, StepId source_step_id, AssetId asset_id,
                                CameraMeasurements const& data) const;

    CameraMeasurements ExtractedTargetsSelect(StepId step_id, AssetId asset_id) const;

    void TargetInfoInsert(StepId step_id, AssetId asset_id, TargetInfo const& target_info) const;

    std::optional<TargetInfo> TargetInfoSelect(StepId step_id, AssetId asset_id) const;

   private:
    sqlite3* db_{nullptr};
};

}  // namespace reprojection::database