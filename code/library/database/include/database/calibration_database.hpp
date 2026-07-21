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

    AssetId GetOrCreateAsset(AssetType type, size_t index, std::string_view name);

    RecordingId GetOrCreateRecording(std::string_view name, std::string_view hash);

    RunId GetOrCreateRun(RecordingId recording_id, std::string_view config);

    // TODO(Jack): The semantics of this step method are so different from the others that we should probably not use
    // the same name. bool: was this a cache hit?
    std::pair<StepId, bool> GetOrCreateStep(std::optional<RecordingId> const& recording_id,
                                            std::optional<RunId> const& run_id, StepType type,
                                            std::string_view cache_key);

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