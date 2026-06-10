#include "steps/pose_initialization.hpp"

#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "optimization/bundle_adjustment.hpp"  // REQUIRED BECAUSE OF ReprojectionResiduals()

namespace reprojection::steps {

std::string PoseInitialization::HashInputs() const {
    return hashing::HashArguments(camera_info_, targets_, intrinsics_);
}

Frames PoseInitialization::Compute() const {
    return calibration::PoseInitialization(camera_info_, targets_, intrinsics_);
}

Frames PoseInitialization::Load(SqlitePtr const db) const { return database::ReadPoses(db, EntityId(), step_type); }

void PoseInitialization::Save(Frames const& initialized_poses, SqlitePtr const db) const {
    database::InsertPoses(db, EntityId(), step_type, initialized_poses);

    OptimizationState const state{intrinsics_, initialized_poses};
    ReprojectionErrors const error{optimization::ReprojectionError(camera_info_, targets_, state)};
    database::InsertReprojectionErrors(db, EntityId(), step_type, error);
}

}  // namespace reprojection::steps
