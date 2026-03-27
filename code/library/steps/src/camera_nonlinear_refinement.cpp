#include "steps/camera_nonlinear_refinement.hpp"

#include "caching/cache_keys.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"

namespace reprojection::application {

std::string CnlrStep::CacheKey() const { return caching::CacheKey(camera_info, targets, initial_state); }

OptimizationState CnlrStep::Compute() const {
    auto const [optimized_state, _]{optimization::CameraNonlinearRefinement(camera_info, targets, initial_state)};

    return optimized_state;
}

OptimizationState CnlrStep::Load(std::shared_ptr<database::CalibrationDatabase const> const db) const {
    Frames const poses{database::ReadPoses(db, step_type, SensorName())};
    auto const intrinsics{database::ReadCameraState(db, step_type, camera_info.sensor_name, camera_info.camera_model)};

    // TODO(Jack): Is this the appropriate error handling? What actual invariants do we have/want here? What if there
    //  are zero poses, is that ok?
    if (not intrinsics.has_value()) {
        throw std::runtime_error(                                                   // LCOV_EXCL_LINE
            "Invalid OptimizationState in CameraNonlinearRefinementStep::Load()");  // LCOV_EXCL_LINE
    }

    return {CameraState{intrinsics.value()}, poses};
}

void CnlrStep::Save(OptimizationState const& optimized_state,
                    std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(optimized_state.camera_state, camera_info.camera_model, step_type, SensorName(), db);
    database::WriteToDb(optimized_state.frames, step_type, SensorName(), db);

    ReprojectionErrors const error{optimization::ReprojectionResiduals(camera_info, targets, optimized_state)};
    database::WriteToDb(error, step_type, SensorName(), db);
}

}  // namespace reprojection::application
