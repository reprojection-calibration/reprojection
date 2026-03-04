#include "application/application.hpp"

#include "caching/cache_keys.hpp"
#include "calibration/linear_pose_initialization.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "database/sensor_data_interface_getters.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"

namespace reprojection::application {

std::pair<CacheStatus, CameraState> FocalLengthInitialization(
    CameraMeasurements const& targets, CameraInfo const& camera_info,
    std::shared_ptr<database::CalibrationDatabase> const database) {
    std::string const step_name{"focal_length_initialization"};  // TODO USE ENUM
    std::string const cache_key{caching::CacheKey(camera_info, targets)};
    CacheStatus const status{CacheState(database, step_name, cache_key)};

    // TODO(Jack): This logic of handling the step table cache key update should be generic for all cases!
    if (status == CacheStatus::StepNameMiss or status == CacheStatus::CacheKeyMiss) {
        std::cout << "calculate the focal length initialization! - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;

        if (status == CacheStatus::StepNameMiss) {
            database::AddCalibrationStep(step_name, cache_key, database);
        } else {
            std::cout << "cache key update - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;
        }

        // ERROR WE JUST RETURN BLANK!
        return {status, {}};
    } else if (status == CacheStatus::CacheHit) {
        auto const result{GetIntrinsic(database, step_name, camera_info.sensor_name)};
        if (not result.has_value()) {
            throw std::runtime_error("We need a real error handling strategy here!");
        }
        auto const [_, camera_state]{result.value()};

        return {status, camera_state};
    } else {
        throw std::runtime_error("Library implementation error - unknown CacheStatus!!!!");
    }
}

std::pair<CacheStatus, OptimizationState> LinearPoseInitialization(
    CameraInfo const& camera_info, CameraMeasurements const& targets, CameraState const& camera_state,
    std::shared_ptr<database::CalibrationDatabase> const database) {
    std::string const step_name{"linear_pose_initialization"};
    std::string const cache_key{caching::CacheKey(camera_info, targets, camera_state)};
    CacheStatus const status{CacheState(database, step_name, cache_key)};

    if (status == CacheStatus::StepNameMiss or status == CacheStatus::CacheKeyMiss) {
        auto const initial_state{calibration::LinearPoseInitialization(camera_info, targets, camera_state)};
        ReprojectionErrors const initial_error{
            optimization::ReprojectionResiduals(camera_info, targets, initial_state)};

        if (status == CacheStatus::StepNameMiss) {
            database::AddCalibrationStep(step_name, cache_key, database);
        } else {
            std::cout << "cache key update - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;
            exit(1);
        }
        database::AddPoseData(initial_state.frames, "linear_pose_initialization", camera_info.sensor_name, database);
        database::AddReprojectionError(initial_error, "linear_pose_initialization", camera_info.sensor_name, database);

        // ERROR WE JUST RETURN BLANK!
        return {status, initial_state};
    } else if (status == CacheStatus::CacheHit) {
        // TODO(Jack): Theoretically we could also load the intrinsics, but as they are passed in via camera_state we
        //  will just use those instead. Does that violate the principle of least suprise? I mean the
        //  calibration::LinearPoseInitialization also just copies camera_state from the input.
        Frames const frames{database::GetPoses(database, step_name, camera_info.sensor_name)};

        return {status, {camera_state, frames}};
    } else {
        throw std::runtime_error("Library implementation error - unknown CacheStatus!!!!");
    }
}

std::pair<CacheStatus, OptimizationState> CameraNonlinearRefinement(
    CameraInfo const& camera_info, CameraMeasurements const& targets, OptimizationState const& optimization_state,
    std::shared_ptr<database::CalibrationDatabase> const database) {
    std::string const step_name{"nonlinear_refinement"};
    std::string const cache_key{caching::CacheKey(camera_info, targets, optimization_state)};
    CacheStatus const status{CacheState(database, step_name, cache_key)};

    if (status == CacheStatus::StepNameMiss or status == CacheStatus::CacheKeyMiss) {
        auto const [optimized_state,
                    _]{optimization::CameraNonlinearRefinement(camera_info, targets, optimization_state)};
        ReprojectionErrors const optimized_error{
            optimization::ReprojectionResiduals(camera_info, targets, optimized_state)};

        if (status == CacheStatus::StepNameMiss) {
            database::AddCalibrationStep(step_name, cache_key, database);
        } else {
            std::cout << "cache key update - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;
            exit(1);
        }
        database::AddPoseData(optimized_state.frames, step_name, camera_info.sensor_name, database);
        database::AddReprojectionError(optimized_error, step_name, camera_info.sensor_name, database);
        database::AddIntrinsics(camera_info, optimization_state.camera_state, step_name, database);

        // ERROR WE JUST RETURN BLANK!
        return {status, optimized_state};
    } else if (status == CacheStatus::CacheHit) {
        auto const loaded_intrinsics{database::GetIntrinsic(database, step_name, camera_info.sensor_name)};
        if (not loaded_intrinsics) {
            throw std::runtime_error("WE NEED  A REAL ERROR HANDLING STRATEGY");
        }

        Frames const frames{database::GetPoses(database, step_name, camera_info.sensor_name)};

        return {status, {loaded_intrinsics->second, frames}};
    } else {
        throw std::runtime_error("Library implementation error - unknown CacheStatus!!!!");
    }
}

// TODO(Jack): Test!
// TODO(Jack): Better name than 'db_cache_key'?
CacheStatus CacheState(std::shared_ptr<database::CalibrationDatabase const> const database, std::string_view step_name,
                       std::string_view cache_key) {
    auto const db_cache_key{database::GetCacheKey(database, step_name)};

    if (not db_cache_key) {
        return CacheStatus::StepNameMiss;
    } else if (db_cache_key.value() == cache_key) {
        return CacheStatus::CacheHit;
    } else {
        return CacheStatus::CacheKeyMiss;
    }
}

}  // namespace reprojection::application