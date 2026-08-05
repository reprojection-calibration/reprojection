#pragma once

#include "database/calibration_database.hpp"

namespace reprojection::steps {

struct SplineInitialization {
    SplineInitialization(AssetId camera_id, StepId camera_poses_id, SqlitePtr db);

    static StepType Type() { return StepType::SplineInit; }

    Hash CacheKey() const;

    void Execute(StepId step_id, SqlitePtr db) const;

   private:
    AssetId camera_id_;
    Frames camera_poses_;
};

}  // namespace reprojection::steps
