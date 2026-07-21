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

    AssetId GetOrCreateAsset(AssetType type, size_t index, Name const& name);

    RecordingId GetOrCreateRecording(Name const& name, Hash const& hash);

    // TODO(Jack): Use config type here!
    RunId GetOrCreateRun(RecordingId recording_id, std::string_view config);

    // TODO(Jack): The semantics of this step method are so different from the others that we should probably not use
    // the same name. bool: was this a cache hit?
    std::pair<StepId, CacheStatus> GetOrCreateStep(std::optional<RecordingId> const& recording_id,
                                                   std::optional<RunId> const& run_id, StepType type, Hash cache_key);

    // NOTE(Jack): We need the step creation and cache key insertion to be separate because if the step execution fails
    // we do not want stale/bad cache keys in the database. By splitting this up and implementing it carefully in the
    // step running logic we can ensure a cache key only gets written if the execution was succesful.
    void StepCacheKeyUpdate(StepId step_id, Hash const& cache_key);

    void ImagesInsert(StepId step_id, AssetId asset_id, EncodedImages const& data);

    EncodedImages ImagesSelect(StepId step_id, AssetId asset_id);

    void ExtractedTargetsInsert(StepId step_id, StepId source_step_id, AssetId asset_id,
                                CameraMeasurements const& data);

    CameraMeasurements ExtractedTargetsSelect(StepId step_id, AssetId asset_id);

    void TargetInfoInsert(StepId step_id, AssetId asset_id, TargetInfo const& target_info);

    std::optional<TargetInfo> TargetInfoSelect(StepId step_id, AssetId asset_id);

   private:
    sqlite3* db_{nullptr};
};

}  // namespace reprojection::database