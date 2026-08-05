#pragma once

#include "types/database_types.hpp"
#include "types/calibration_types.hpp"
#include "database/calibration_database.hpp"

namespace reprojection::steps {

struct PoseInitialization {
    PoseInitialization(AssetId camera_id, StepId targets_id, StepId camera_info_id, StepId intrinsics_id,
                       database::CalibrationDatabase const& db);

    static StepType Type() { return StepType::PoseInit; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase const& db) const;

   private:
    AssetId camera_id_;
    StepId targets_id_;
    CameraMeasurements targets_;
    CameraInfo camera_info_;
    CameraState intrinsics_;
};

}  // namespace reprojection::steps
