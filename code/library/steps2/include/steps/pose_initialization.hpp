#pragma once

#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"

namespace reprojection::steps {

struct PoseInitialization {
    PoseInitialization(AssetId camera_id, StepId targets_id, StepId camera_info_id, StepId intrinsics_id,
                       database::CalibrationDatabase& db);

    static StepType Type() { return StepType::PoseInitialization; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:
    AssetId camera_id_;
    StepId targets_id_;
    CameraMeasurements targets_;
    CameraInfo camera_info_;
    CameraState intrinsics_;
};

}  // namespace reprojection::steps
