#include "optimization/bundle_adjustment.hpp"

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "steps/bundle_adjustment.hpp"

namespace reprojection::steps {

std::string BundleAdjustment::CacheKey() const { return caching::HashArguments(camera_info, targets, initial_state); }

OptimizationState BundleAdjustment::Compute() const {
    auto const [optimized_state, _]{optimization::BundleAdjustment(camera_info, targets, initial_state)};

    return optimized_state;
}

OptimizationState BundleAdjustment::Load(SqlitePtr const db) const {
    Frames const poses{database::ReadPoses(db, SensorName(), step_type)};
    auto const intrinsics{database::ReadIntrinsics(db, camera_info.sensor_name, step_type, camera_info.camera_model)};

    // TODO(Jack): Is this the appropriate error handling? What actual invariants do we have/want here? What if there
    //  are zero poses, is that ok?
    if (not intrinsics.has_value()) {
        throw std::runtime_error(                                          // LCOV_EXCL_LINE
            "Invalid OptimizationState in BundleAdjustmentStep::Load()");  // LCOV_EXCL_LINE
    }

    return {CameraState{intrinsics.value()}, poses};
}

void BundleAdjustment::Save(OptimizationState const& optimized_state, SqlitePtr const db) const {
    database::InsertIntrinsics(db, SensorName(), step_type, camera_info.camera_model, optimized_state.camera_state);
    database::InsertPoses(db, SensorName(), step_type, optimized_state.frames);

    ReprojectionErrors const error{optimization::ReprojectionError(camera_info, targets, optimized_state)};
    database::InsertReprojectionErrors(db, SensorName(), step_type, error);
}

}  // namespace reprojection::steps
