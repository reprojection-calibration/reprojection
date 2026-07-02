#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct BundleAdjustment {
    CameraInfo camera_info_;
    CameraMeasurements targets_;
    OptimizationState initial_state_;
    int num_threads_;

    static CalibrationStep StepType() { return CalibrationStep::BundleAdjustment; }

    std::string EntityId() const { return camera_info_.sensor_name; }

    std::string HashInputs() const;

    OptimizationState Compute() const;

    OptimizationState Load(SqlitePtr const db) const;

    void Save(OptimizationState const& optimized_state, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
