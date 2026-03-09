

#include "../include/application/steps.hpp"

#include "caching/cache_keys.hpp"
#include "calibration/linear_pose_initialization.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"

namespace reprojection::application {

LinearPoseInitializationStep::LinearPoseInitializationStep(CameraInfo const& _camera_info,
                                                           CameraMeasurements const& _targets,
                                                           CameraState const& _camera_state)
    : camera_info{_camera_info}, targets{_targets}, camera_state{_camera_state} {}

std::string LinearPoseInitializationStep::CacheKey() const {
    return caching::CacheKey(camera_info, targets, camera_state);
}

Frames LinearPoseInitializationStep::Compute() const {
    return calibration::LinearPoseInitialization(camera_info, targets, camera_state);
}

Frames LinearPoseInitializationStep::Load(std::shared_ptr<database::CalibrationDatabase const> const db) const {
    return database::ReadPoses(db, step_type, SensorName());
}

void LinearPoseInitializationStep::Save(Frames const& frames,
                                        std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(frames, step_type, SensorName(), db);

    OptimizationState const state{camera_state, frames};
    ReprojectionErrors const error{optimization::ReprojectionResiduals(camera_info, targets, state)};
    database::WriteToDb(error, step_type, SensorName(), db);
}

}  // namespace reprojection::application
