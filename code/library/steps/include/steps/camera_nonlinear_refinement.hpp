#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct CnlrStep {
    CameraInfo camera_info;
    CameraMeasurements targets;
    OptimizationState initial_state;

    CalibrationStep step_type{CalibrationStep::Cnlr};

    std::string SensorName() const { return camera_info.sensor_name; }

    std::string CacheKey() const;

    OptimizationState Compute() const;

    OptimizationState Load(SqlitePtr const db) const;

    void Save(OptimizationState const& optimized_state, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
