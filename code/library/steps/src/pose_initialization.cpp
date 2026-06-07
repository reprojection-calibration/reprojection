#include "steps/pose_initialization.hpp"

#include "hashing/hashing.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/bundle_adjustment.hpp"  // REQUIRED BECAUSE OF ReprojectionResiduals()

namespace reprojection::steps {

std::string PoseInitialization::CacheKey() const { return hashing::HashArguments(camera_info, targets, camera_state); }

Frames PoseInitialization::Compute() const {
    return calibration::PoseInitialization(camera_info, targets, camera_state);
}

Frames PoseInitialization::Load(SqlitePtr const db) const { return database::ReadPoses(db, SensorName(), step_type); }

void PoseInitialization::Save(Frames const& frames, SqlitePtr const db) const {
    database::InsertPoses(db, SensorName(), step_type, frames);

    OptimizationState const state{camera_state, frames};
    ReprojectionErrors const error{optimization::ReprojectionError(camera_info, targets, state)};
    database::InsertReprojectionErrors(db, SensorName(), step_type, error);
}

}  // namespace reprojection::steps
