#pragma once

#include "types/calibration_types.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

struct PoseInitialization {
    PoseInitialization(AssetId camera_id, StepId targets_id, StepId camera_info_id, StepId intrinsics_id, SqlitePtr db);

    static StepType Type() { return StepType::PoseInit; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    StepId targets_id_;
    CameraMeasurements targets_;
    CameraInfo camera_info_;
    CameraState intrinsics_;
};

}  // namespace reprojection::steps
