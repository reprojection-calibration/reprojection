#pragma once

#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"

namespace reprojection::steps {

struct SplineInitialization {
    SplineInitialization(AssetId camera_id, StepId camera_poses_id, database::CalibrationDatabase const& db);

    static StepType Type() { return StepType::SplineInit; }

    Hash CacheKey() const;

    void Execute(StepId step_id, database::CalibrationDatabase& db) const;

   private:
    AssetId camera_id_;
    Frames camera_poses_;
};

}  // namespace reprojection::steps
