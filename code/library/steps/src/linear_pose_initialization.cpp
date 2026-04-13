#include "steps/linear_pose_initialization.hpp"

#include "caching/cache_keys.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"  // REQUIRED BECAUSE OF ReprojectionResiduals()

namespace reprojection::steps {

std::string LpiStep::CacheKey() const { return caching::CacheKey(camera_info, targets, camera_state); }

Frames LpiStep::Compute() const { return calibration::LinearPoseInitialization(camera_info, targets, camera_state); }

Frames LpiStep::Load(SqlitePtr const& db) const { return database::ReadPoses(db, step_type, SensorName()); }

void LpiStep::Save(Frames const& frames, SqlitePtr const& db) const {
    database::WriteToDb(frames, step_type, SensorName(), db);

    OptimizationState const state{camera_state, frames};
    ReprojectionErrors const error{optimization::ReprojectionResiduals(camera_info, targets, state)};
    database::WriteToDb(error, step_type, SensorName(), db);
}

}  // namespace reprojection::steps
