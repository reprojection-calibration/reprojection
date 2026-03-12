#include "application/steps.hpp"

#include "caching/cache_keys.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"

namespace reprojection::application {

IiStep::IiStep(CameraInfo const& _camera_info, CameraMeasurements const& _targets)
    : camera_info{_camera_info}, targets{_targets} {}

std::string IiStep::CacheKey() const { return caching::CacheKey(camera_info, targets); }

CameraState IiStep::Compute() const {
    // TODO(Jack): Confirm v and u are height and width in the correct order!
    auto const intrinsics{calibration::InitializeIntrinsics(camera_info.camera_model, camera_info.bounds.v_max,
                                                            camera_info.bounds.u_max, targets)};

    if (not intrinsics.has_value()) {
        throw std::runtime_error("We have no error handling strategy for failed IiStep::Compute()");
    }

    return CameraState{*intrinsics};
}

CameraState IiStep::Load(std::shared_ptr<database::CalibrationDatabase const> const db) const {
    auto const loaded_intrinsics{
        database::ReadCameraState(db, step_type, camera_info.sensor_name, camera_info.camera_model)};

    if (not loaded_intrinsics.has_value()) {
        throw std::runtime_error("We have no error handling strategy for failed IiStep::Load()");
    }

    return CameraState{*loaded_intrinsics};
}

void IiStep::Save(CameraState const& intrinsics, std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(intrinsics, camera_info.camera_model, step_type, camera_info.sensor_name, db);
}

LpiStep::LpiStep(CameraInfo const& _camera_info, CameraMeasurements const& _targets, CameraState const& _camera_state)
    : camera_info{_camera_info}, targets{_targets}, camera_state{_camera_state} {}

std::string LpiStep::CacheKey() const { return caching::CacheKey(camera_info, targets, camera_state); }

Frames LpiStep::Compute() const { return calibration::LinearPoseInitialization(camera_info, targets, camera_state); }

Frames LpiStep::Load(std::shared_ptr<database::CalibrationDatabase const> const db) const {
    return database::ReadPoses(db, step_type, SensorName());
}

void LpiStep::Save(Frames const& frames, std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(frames, step_type, SensorName(), db);

    OptimizationState const state{camera_state, frames};
    ReprojectionErrors const error{optimization::ReprojectionResiduals(camera_info, targets, state)};
    database::WriteToDb(error, step_type, SensorName(), db);
}

CnlrStep::CnlrStep(CameraInfo const& _camera_info, CameraMeasurements const& _targets,
                   OptimizationState const& _initial_state)
    : camera_info{_camera_info}, targets{_targets}, initial_state{_initial_state} {}

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
