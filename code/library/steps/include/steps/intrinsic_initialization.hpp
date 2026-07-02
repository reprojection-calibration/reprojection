#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct IntrinsicInitialization {
    CameraInfo camera_info_;
    CameraMeasurements targets_;
    int num_threads_;

    static CalibrationStep StepType() { return CalibrationStep::IntrinsicInitialization; }

    std::string EntityId() const { return camera_info_.sensor_name; }

    std::string HashInputs() const;

    CameraState Compute() const;

    CameraState Load(SqlitePtr const db) const;

    void Save(CameraState const& frames, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
