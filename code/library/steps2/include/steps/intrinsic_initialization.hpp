#pragma once

#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"

namespace reprojection::steps {

struct IntrinsicInitialization {
    IntrinsicInitialization(AssetId camera_id, int num_threads, StepId camera_info_id, StepId targets_id, database::CalibrationDatabase& db);

    static StepType Type() { return StepType::IntrinsicInitialization; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:
    AssetId camera_id_;
    int num_threads_;
    CameraInfo camera_info_;
    CameraMeasurements targets_;
};

}  // namespace reprojection::steps
