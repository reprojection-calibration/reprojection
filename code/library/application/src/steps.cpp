#include "application/steps.hpp"

#include "caching/cache_keys.hpp"
#include "calibration/initialization_methods.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "feature_extraction/target_extraction.hpp"
#include "optimization/camera_nonlinear_refinement.hpp"

namespace reprojection::application {

std::string FeatureExtractionStep::CacheKey() const { return cache_key; }

CameraMeasurements FeatureExtractionStep::Compute() const {
    // TODO(Jack): Is it really appropriate to use a toml table here instead of a struct?
    auto const extractor{feature_extraction::CreateTargetExtractor(target_config)};

    CameraMeasurements extracted_targets;
    while (auto const data{image_source()}) {
        auto const& [timestamp_ns, image]{*data};
        // TODO(Jack): Does the format of the image matter? gray or rgb etc?
        std::optional<ExtractedTarget> const target{extractor->Extract(image)};
        if (target.has_value()) {
            extracted_targets.insert({timestamp_ns, target.value()});
        }
    }

    return extracted_targets;
}

CameraMeasurements FeatureExtractionStep::Load(std::shared_ptr<database::CalibrationDatabase const> const db) const {
    return database::GetExtractedTargetData(db, SensorName());
}

void FeatureExtractionStep::Save(CameraMeasurements const& extracted_targets,
                                 std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(extracted_targets, SensorName(), db);
}

std::string IntrinsicInitializationStep::CacheKey() const { return caching::CacheKey(camera_info, targets); }

CameraState IntrinsicInitializationStep::Compute() const {
    // TODO(Jack): Confirm v and u are height and width in the correct order!
    auto const intrinsics{calibration::InitializeIntrinsics(camera_info.camera_model, camera_info.bounds.v_max,
                                                            camera_info.bounds.u_max, targets)};

    if (not intrinsics.has_value()) {
        throw std::runtime_error("We have no error handling strategy for failed IiStep::Compute()");  // LCOV_EXCL_LINE
    }

    return CameraState{*intrinsics};
}

CameraState IntrinsicInitializationStep::Load(std::shared_ptr<database::CalibrationDatabase const> const db) const {
    auto const loaded_intrinsics{
        database::ReadCameraState(db, step_type, camera_info.sensor_name, camera_info.camera_model)};

    if (not loaded_intrinsics.has_value()) {
        throw std::runtime_error("We have no error handling strategy for failed IiStep::Load()");  // LCOV_EXCL_LINE
    }

    return CameraState{*loaded_intrinsics};
}

void IntrinsicInitializationStep::Save(CameraState const& intrinsics,
                                       std::shared_ptr<database::CalibrationDatabase> const db) const {
    database::WriteToDb(intrinsics, camera_info.camera_model, step_type, camera_info.sensor_name, db);
}

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
