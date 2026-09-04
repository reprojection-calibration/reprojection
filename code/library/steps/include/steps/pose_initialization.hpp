#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct PoseInitialization {
    PoseInitialization(AssetId camera_id, StepId targets_id, StepId camera_info_id, StepId intrinsics_id, SqlitePtr db);

    static StepType Type() { return StepType::PoseInit; }

    std::vector<AssetId> Assets() const { return {camera_id_}; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    StepId targets_id_;
    CameraMeasurements targets_;
    CameraInfo camera_info_;
    Intrinsic intrinsics_;
};

}  // namespace reprojection::steps
