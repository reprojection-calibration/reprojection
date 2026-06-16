#pragma once

#include "database/calibration_database.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::steps {

struct PoseInitialization {
    CameraInfo camera_info_;
    CameraMeasurements targets_;
    CameraState camera_state_;

    static CalibrationStep StepType() { return CalibrationStep::PoseInitialization; }

    std::string EntityId() const { return camera_info_.sensor_name; }

    std::string HashInputs() const;

    Frames Compute() const;

    Frames Load(SqlitePtr const db) const;

    void Save(Frames const& frames, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
