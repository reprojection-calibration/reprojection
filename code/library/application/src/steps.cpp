

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

CameraNonlinearRefinementStep::CameraNonlinearRefinementStep(CameraInfo const& _camera_info,
                                                             CameraMeasurements const& _targets,
                                                             OptimizationState const& _initial_state)
    : camera_info{_camera_info}, targets{_targets}, initial_state{_initial_state} {}

std::string CameraNonlinearRefinementStep::CacheKey() const {
    return caching::CacheKey(camera_info, targets, initial_state);
}

OptimizationState CameraNonlinearRefinementStep::Compute() const {
    auto const [optimized_state, _]{optimization::CameraNonlinearRefinement(camera_info, targets, initial_state)};

    return optimized_state;
}

OptimizationState CameraNonlinearRefinementStep::Load(
    std::shared_ptr<database::CalibrationDatabase const> const db) const {
    Frames const poses{database::ReadPoses(db, step_type, SensorName())};
    auto const intrinsics{database::ReadCameraState(db, step_type, camera_info.sensor_name, camera_info.camera_model)};

    // TODO(Jack): Is this the appropriate error handling? What actual invariants do we have/want here?
    if (not intrinsics.has_value()) {
        throw std::runtime_error("Invalid OptimizationState in CameraNonlinearRefinementStep::Load()");
    }

    return {CameraState{intrinsics.value()}, poses};
}

void CameraNonlinearRefinementStep::Save(OptimizationState const& optimized_state,
                                         std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(optimized_state.frames, step_type, SensorName(), db);
    database::WriteToDb(optimized_state.camera_state, camera_info.camera_model, step_type, SensorName(), db);

    ReprojectionErrors const error{optimization::ReprojectionResiduals(camera_info, targets, optimized_state)};
    database::WriteToDb(error, step_type, SensorName(), db);
}

}  // namespace reprojection::application
