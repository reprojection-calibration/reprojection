#include "optimization/bundle_adjustment.hpp"

#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "steps/bundle_adjustment.hpp"

namespace reprojection::steps {

std::string BundleAdjustment::HashInputs() const {
    return hashing::HashArguments(camera_info_, targets_, initial_state_);
}

OptimizationState BundleAdjustment::Compute() const {
    auto const [optimized_state, _]{optimization::BundleAdjustment(camera_info_, targets_, initial_state_)};

    return optimized_state;
}

OptimizationState BundleAdjustment::Load(SqlitePtr const db) const {
    Frames const poses{database::ReadPoses(db, EntityId(), StepType())};
    auto const intrinsics{database::ReadIntrinsics(db, EntityId(), StepType(), camera_info_.camera_model)};

    // TODO(Jack): Is this the appropriate error handling? What actual invariants do we have/want here? What if there
    //  are zero poses, is that ok?
    if (not intrinsics.has_value()) {
        throw std::runtime_error(                                          // LCOV_EXCL_LINE
            "Invalid OptimizationState in BundleAdjustmentStep::Load()");  // LCOV_EXCL_LINE
    }

    return {CameraState{intrinsics.value()}, poses};
}

void BundleAdjustment::Save(OptimizationState const& optimized_state, SqlitePtr const db) const {
    database::InsertIntrinsics(db, EntityId(), StepType(), camera_info_.camera_model, optimized_state.camera_state);
    database::InsertPoses(db, EntityId(), StepType(), optimized_state.frames);

    ReprojectionErrors const error{optimization::ReprojectionError(camera_info_, targets_, optimized_state)};
    database::InsertReprojectionErrors(db, EntityId(), StepType(), error);
}

}  // namespace reprojection::steps
