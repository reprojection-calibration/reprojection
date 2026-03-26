#pragma once

#include <toml++/toml.hpp>

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::application {

struct IntrinsicInitializationStep {
    CameraInfo camera_info;
    CameraMeasurements targets;

    CalibrationStep step_type{CalibrationStep::Ii};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    CameraState Compute() const;

    CameraState Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(CameraState const& frames, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

// TODO(Jack): Make private package source files one day when the application is whole.
struct LpiStep {
    CameraInfo camera_info;
    CameraMeasurements targets;
    CameraState camera_state;

    CalibrationStep step_type{CalibrationStep::Lpi};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    Frames Compute() const;

    Frames Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(Frames const& frames, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

struct CnlrStep {
    CameraInfo camera_info;
    CameraMeasurements targets;
    OptimizationState initial_state;

    CalibrationStep step_type{CalibrationStep::Cnlr};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    OptimizationState Compute() const;

    OptimizationState Load(std::shared_ptr<database::CalibrationDatabase const> const db) const;

    void Save(OptimizationState const& optimized_state, std::shared_ptr<database::CalibrationDatabase> const db) const;
};

}  // namespace reprojection::application
