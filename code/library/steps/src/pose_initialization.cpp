#include "steps/pose_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "optimization/bundle_adjustment.hpp"  // REQUIRED BECAUSE OF ReprojectionResiduals()

namespace reprojection::steps {

std::string PoseInitialization::HashInputs() const {
    return hashing::HashArguments(camera_info_, targets_, camera_state_);
}

Frames PoseInitialization::Compute() const {
    return calibration::PoseInitialization(camera_info_, targets_, camera_state_);
}

Frames PoseInitialization::Load(SqlitePtr const db) const { return database::ReadPoses(db, EntityId(), step_type); }

void PoseInitialization::Save(Frames const& frames, SqlitePtr const db) const {
    database::InsertPoses(db, EntityId(), step_type, frames);

    OptimizationState const state{camera_state_, frames};
    ReprojectionErrors const error{optimization::ReprojectionError(camera_info_, targets_, state)};
    database::InsertReprojectionErrors(db, EntityId(), step_type, error);
}

}  // namespace reprojection::steps
