#include "caching/cache_keys.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/bundle_adjustment.hpp"  // REQUIRED BECAUSE OF ReprojectionResiduals()
#include "steps/pose_initialization.hpp"

namespace reprojection::steps {

std::string PoseInitialization::CacheKey() const { return caching::CacheKey(camera_info, targets, camera_state); }

Frames PoseInitialization::Compute() const { return calibration::PoseInitialization(camera_info, targets, camera_state); }

Frames PoseInitialization::Load(SqlitePtr const db) const { return database::ReadPoses(db, step_type, SensorName()); }

void PoseInitialization::Save(Frames const& frames, SqlitePtr const db) const {
    database::WriteToDb(frames, step_type, SensorName(), db);

    OptimizationState const state{camera_state, frames};
    ReprojectionErrors const error{optimization::ReprojectionError(camera_info, targets, state)};
    database::WriteToDb(error, step_type, SensorName(), db);
}

}  // namespace reprojection::steps
